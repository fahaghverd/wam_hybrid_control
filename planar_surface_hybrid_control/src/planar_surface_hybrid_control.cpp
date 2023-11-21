/*
 * planar_surface_hybrid_control.cpp
 *
 *  
 * 
 * Created on: Nov., 2023
 * Author: Faezeh
 */

#include "planar_surface_hybrid_control/planar_surface_hybrid_control.h"

// Templated Initialization Function
template<size_t DOF>
void PlanarHybridControl<DOF>::init(ProductManager& pm)
{
    msg_timeout.fromSec(0.3);
    mypm = &pm;

    locked_joints = false;
  
    ROS_INFO("%zu-DOF WAM", DOF);
    jp_home = wam.getJointPositions();
    wam.gravityCompensate(true); // gravity compensation default set to true
    
    // setting up WAM joint state publisher
    const char* wam_jnts[] = {  "wam/YawJoint",
                                "wam/ShoulderPitchJoint",
                                "wam/ShoulderYawJoint",
                                "wam/ElbowJoint",
                                "wam/UpperWristYawJoint",
                                "wam/UpperWristPitchJoint",
                                "wam/LowerWristYawJoint"
                             };
    std::vector < std::string > wam_joints(wam_jnts, wam_jnts + 7);
    wam_joint_state.name = wam_joints;
    wam_joint_state.name.resize(DOF);
    wam_joint_state.position.resize(DOF);
    wam_joint_state.velocity.resize(DOF);
    wam_joint_state.effort.resize(DOF);
    wam_jacobian_mn.data.resize(DOF*6);

    // ros services
    go_home_srv = n_.advertiseService("go_home", &PlanarHybridControl::go_home_callback, this);
    joint_move_block_srv = n_.advertiseService("joint_move_block", &PlanarHybridControl::joint_move_block_callback, this);
    surface_calibrartion_srv = n_.advertiseService("surface_calibrartion", &PlanarHybridControl<DOF>::calibration, this);
    collect_cp_trajectory_srv = n_.advertiseService("collect_cp_trajectory", &PlanarHybridControl<DOF>::collectCpTrajectory, this);
    planar_surface_hybrid_control_srv = n_.advertiseService("planar_surface_hybrid_control", &PlanarHybridControl<DOF>::HybridCartImpForceCOntroller, this);
    
    //ros publishers
    wam_joint_state_pub = n_.advertise < sensor_msgs::JointState > ("joint_states", 1);
    wam_pose_pub = n_.advertise < geometry_msgs::PoseStamped > ("pose", 1);
    wam_jacobian_mn_pub = n_.advertise < wam_msgs::MatrixMN > ("jacobian",1);
    wam_tool_pub = n_.advertise < wam_msgs::RTToolInfo > ("tool_info",1);
    wam_estimated_contact_force_pub = n_.advertise < wam_msgs::RTCartForce > ("static_estimated_force",1);
    
    // ros subscribers
    
    ROS_INFO("wam services now advertised");
    ros::AsyncSpinner spinner(0);
    spinner.start();
}

// Templated surface calibration Function
template<size_t DOF>
bool PlanarHybridControl<DOF>::calibration(wam_srvs::Teach::Request &req, wam_srvs::Teach::Response &res)
{   
    cp_type v1, v2, v3;
    systems::Ramp time(mypm->getExecutionManager());
    systems::TupleGrouper<double, cp_type, jp_type> configLogTg;
    const double T_s = mypm->getExecutionManager()->getPeriod();
    std::string path = "/home/catkin_workspace/src/wam_hybrid_control/.data/" + req.path;

    // Record at 1/10th of the loop rate
    systems::PeriodicDataLogger<config_sample_type> configLogger(mypm->getExecutionManager(), new barrett::log::RealTimeWriter<config_sample_type>(path.c_str(), 10*T_s), 10);

    std::cout<< "Move the robot to the first contact point and Press [Enter]."<<std::endl;
    waitForEnter();
    v1 = wam.getToolPosition();
    initial_point = v1;
    {   
        // Make sure the Systems are connected on the same execution cycle
        // that the time is started. Otherwise we might record a bunch of
        // samples all having t=0; this is bad because the Spline requires time
        // to be monotonic.
        BARRETT_SCOPED_LOCK(mypm->getExecutionManager()->getMutex());
        connect(time.output, configLogTg.template getInput<0>());
        connect(wam.toolPosition.output, configLogTg.template getInput<1>());
        connect(wam.jpOutput, configLogTg.template getInput<2>());
        connect(configLogTg.output, configLogger.input);
        time.start();
    }

    std::cout<< "Move the robot on the surface in a line and Press [Enter]."<<std::endl;
    waitForEnter();
    v2 = wam.getToolPosition();

    std::cout<< "Move the robot on the surface in a line perpendicular to the previous line and Press [Enter]."<<std::endl;
    waitForEnter();
    v3 = wam.getToolPosition();

    configLogger.closeLog();
    disconnect(configLogger.input);

    surface_normal = ((v2-v1).cross(v3-v2));
    surface_normal.normalize();
    std::cout<<"Surface normal: "<< surface_normal << std::endl;   

    return true;
}

/*// Templated Function to find linearly independent vectors in the collected trajectory.
template<size_t DOF>
void PlanarHybridControl<DOF>::findLinearlyIndependentVectors(ProductManager& pm)
{   
    cp_type v1, v2, v3;
    std::string path = "/home/.data/motions/trj.text";
    log::Reader<config_sample_type> lr(path.c_str());
    std::vector<config_sample_type> vec;

    for (size_t i = 0; i < lr.numRecords(); ++i) {
        vec.push_back(lr.getRecord());
    }

    // Extract cp_type values from vec
    std::vector<cp_type> cp_values, normals;

    for (const auto& record : vec) {
        cp_values.push_back(boost::get<1>(record)); // Assuming cp_type is at index 1
    }

    cp_type pre_p = cp_values[i]; //previous point that we are finding the normal on it.
    for (const auto& cp_value : cp_values) {
        if((pre_cp - cp_value).norm > 0.1){
            std::cout << cp_value << " ";


        }        
    }
}*/

//Function to teach a motion to the WAM
template<size_t DOF>
bool PlanarHybridControl<DOF>::collectCpTrajectory(wam_srvs::Teach::Request &req, wam_srvs::Teach::Response &res) {
    ROS_INFO("Collecting cartesian trajectory.");
    systems::Ramp time(mypm->getExecutionManager());
    systems::TupleGrouper<double, cp_type> cpLogTg;
    const double T_s = mypm->getExecutionManager()->getPeriod();
    std::string path = "/home/catkin_workspace/src/wam_hybrid_control/.data/" + req.path;
    ROS_INFO_STREAM("Collecting cartesian trajectory. Saving to: " << path);
    // Record at 1/10th of the loop rate
    systems::PeriodicDataLogger<cp_sample_type> cpLogger(mypm->getExecutionManager(), new barrett::log::RealTimeWriter<cp_sample_type>(path.c_str(), 10*T_s), 10);
    printf("Press [Enter] to start collecting.\n");
    waitForEnter();
    {
        // Make sure the Systems are connected on the same execution cycle
        // that the time is started. Otherwise we might record a bunch of
        // samples all having t=0; this is bad because the Spline requires time
        // to be monotonic.
        BARRETT_SCOPED_LOCK(mypm->getExecutionManager()->getMutex());
        connect(time.output, cpLogTg.template getInput<0>());
        connect(wam.toolPosition.output, cpLogTg.template getInput<1>());
        connect(cpLogTg.output, cpLogger.input);
        time.start();
    }
    printf("Press [Enter] to stop collecting.\n");
    waitForEnter();
    cpLogger.closeLog();
    disconnect(cpLogger.input);
    ROS_INFO_STREAM("Collecting done.");
    return true;
}

// Hybrid force + impedance cp_position controller
//Lets play the collected trajectory as the trajectory! We cal also try collecting when not in contaact
//and then it projects. Also, we can reshape the trajectory based on the initial point and the projection
//matirx, like if its drawing z on the wall, should be able to draw it on the table as well.
template<size_t DOF>
bool PlanarHybridControl<DOF>::HybridCartImpForceCOntroller(wam_srvs::Play::Request &req, wam_srvs::Play::Response &res){
    //disconnectSystems();
    //Extracting cartesian trajectory from collected trajectory.
    std::string path = "/home/catkin_workspace/src/wam_hybrid_control/.data/" + req.path;
    log::Reader<config_sample_type> lr(path.c_str());
    std::vector<config_sample_type> vec;

    for (size_t i = 0; i < lr.numRecords(); ++i) {
        vec.push_back(lr.getRecord());
    }

    // Extract cp_type values from vec
    std::vector<cp_type> cp_trj;

    for (const auto& record : vec) {
        cp_trj.push_back(boost::get<1>(record)); // Assuming cp_type is at index 1
    }
    
    //Force estimation
    systems::connect(wam.kinematicsBase.kinOutput, getWAMJacobian.kinInput);
    systems::connect(getWAMJacobian.output, staticForceEstimator.Jacobian);

    systems::connect(wam.kinematicsBase.kinOutput, gravityTerm.kinInput);
    systems::connect(gravityTerm.output, staticForceEstimator.g);

    systems::connect(wam.jtSum.output, staticForceEstimator.jtInput);
   
    //Impedance Control
    jt_type jtLimits(30.0);
    cp_type cp_cmd, KpApplied, KdApplied;
    KpApplied << 300, 250, 200;
    KdApplied << 50, 50, 50;

    KxSet.setValue(KpApplied);
    DxSet.setValue(KdApplied);
    XdSet.setValue(wam.getToolPosition());
    OrnKxSet.setValue(cp_type(0.0, 0.0, 0.0));
    OrnDxSet.setValue(cp_type(0.0, 0.0, 0.0));
    OrnXdSet.setValue(wam.getToolOrientation());

    // CONNECT SPRING SYSTEM
    systems::forceConnect(KxSet.output, ImpControl.KxInput);
    systems::forceConnect(DxSet.output, ImpControl.DxInput);
    systems::forceConnect(XdSet.output, ImpControl.XdInput);

    systems::forceConnect(OrnKxSet.output, ImpControl.OrnKpGains);
    systems::forceConnect(OrnDxSet.output, ImpControl.OrnKdGains);
    systems::forceConnect(OrnXdSet.output, ImpControl.OrnReferenceInput);
    systems::forceConnect(wam.toolOrientation.output, ImpControl.OrnFeedbackInput);

    systems::forceConnect(wam.toolPosition.output, ImpControl.CpInput);
    systems::forceConnect(wam.toolVelocity.output, ImpControl.CvInput);
    systems::forceConnect(wam.kinematicsBase.kinOutput, ImpControl.kinInput);

    systems::forceConnect(wam.kinematicsBase.kinOutput, toolforce2jt.kinInput);
    //systems::forceConnect(wam.kinematicsBase.kinOutput, tooltorque2jt.kinInput);
    systems::forceConnect(wam.kinematicsBase.kinOutput, tt2jt_ortn_split.kinInput); // how tt2jt_ortn_split is different from tooltorque2jt??

    systems::forceConnect(ImpControl.CFOutput, toolforce2jt.input);
    systems::forceConnect(ImpControl.CTOutput, tt2jt_ortn_split.input);

    // CONNECT TO SUMMER
    systems::forceConnect(toolforce2jt.output, torqueSum.getInput(0));
    systems::forceConnect(tt2jt_ortn_split.output, torqueSum.getInput(1));

    // SATURATE AND CONNECT TO WAM INPUT
    systems::forceConnect(torqueSum.output, jtSat.input);        
    systems::forceConnect(jtSat.output, wam.input); 
    
    cp_type projected_waypoint;
    for (const auto& waypoint : cp_trj) {
        // Calculate the vector from the point on the plane to the given point
        cp_type PQ = waypoint - initial_point;

        // Calculate the projection of PQ onto the plane's normal vector
        double projectionScalar = PQ.dot(surface_normal) / surface_normal.squaredNorm();
        cp_type projection = projectionScalar * surface_normal;

        // Move to the waypoint
        projected_waypoint = waypoint - projection;
        XdSet.setValue(projected_waypoint);
        btsleep(0.5);
        cp_type e = (projected_waypoint - wam.getToolPosition())/(projected_waypoint.norm());
        if(e.norm() > 0.05) {std::cout<<e<<std::endl;}   
    }

    systems::disconnect(torqueSum.output);

    return true;
}

// goHome Function for sending the WAM safely back to its home starting position.
template<size_t DOF>
bool PlanarHybridControl<DOF>::go_home_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Returning to Home Position");
    for (size_t i = 0; i < DOF; i++) {
        jp_cmd[i] = 0.0;
    }
    wam.moveTo(jp_cmd, true);
    jp_home[3] -= 0.3;
    wam.moveTo(jp_home, true);
    jp_home[3] += 0.3;
    wam.moveTo(jp_home, true);
    locked_joints = true;
    return true;
}

//Function to command a joint space move to the WAM with blocking specified
template<size_t DOF>
bool PlanarHybridControl<DOF>::joint_move_block_callback(wam_srvs::JointMoveBlock::Request &req, wam_srvs::JointMoveBlock::Response &res)
{
    if (req.joints.size() != DOF) {
        ROS_INFO("Request Failed: %zu-DOF request received, must be %zu-DOF", req.joints.size(), DOF);
        return false;
    }
    ROS_INFO("Moving Robot to Commanded Joint Pose");
    for (size_t i = 0; i < DOF; i++) {
        jp_cmd[i] = req.joints[i];
    }
    bool b = req.blocking;
    wam.moveTo(jp_cmd, b);
    return true;
}

//Function to update the WAM publisher
template<size_t DOF>
void PlanarHybridControl<DOF>::publish_wam(ProductManager& pm)
{
    //Current values to be published
    jp_type jp = wam.getJointPositions();
    jt_type jt = wam.getJointTorques();
    jv_type jv = wam.getJointVelocities();
    cp_type cp_pub = wam.getToolPosition();
    cv_type cv_pub = wam.getToolVelocity();
    cv_type cv_ang;
    cv_ang << cv_pub[3], cv_pub[4], cv_pub[5];
    // std::cout << cv_ang[0] << " " << cv_ang[1] << " " << cv_ang[2] << std::endl;
    Eigen::Quaterniond to_pub = wam.getToolOrientation();
    math::Matrix<6,DOF> robot_tool_jacobian=wam.getToolJacobian();
    //publishing sensor_msgs/JointState to wam/joint_states
    for (size_t i = 0; i < DOF; i++)
    {
        wam_joint_state.position[i] = jp[i];
        wam_joint_state.velocity[i] = jv[i];
        wam_joint_state.effort[i] = jt[i];
    }
    wam_joint_state.header.stamp = ros::Time::now();
    wam_joint_state_pub.publish(wam_joint_state);
    //publishing geometry_msgs/PoseStamed to wam/pose
    wam_pose.header.stamp = ros::Time::now();
    wam_pose.pose.position.x = cp_pub[0];
    wam_pose.pose.position.y = cp_pub[1];
    wam_pose.pose.position.z = cp_pub[2];
    wam_pose.pose.orientation.w = to_pub.w();
    wam_pose.pose.orientation.x = to_pub.x();
    wam_pose.pose.orientation.y = to_pub.y();
    wam_pose.pose.orientation.z = to_pub.z();
    wam_pose_pub.publish(wam_pose);
    //publishing wam_msgs/MatrixMN to wam/jacobian
    wam_jacobian_mn.m = 6;
    wam_jacobian_mn.n = DOF;
    //size_t index_loop=0;
    for (size_t h = 0; h < wam_jacobian_mn.n; ++h) {
        for (size_t k = 0; k < wam_jacobian_mn.m; ++k) {
            wam_jacobian_mn.data[h*6+k]=robot_tool_jacobian(k,h);
        }
    }
    wam_jacobian_mn_pub.publish(wam_jacobian_mn);

    for (size_t j = 0; j < 3; j++) {
        wam_tool_info.position[j] = cp_pub[j];
        wam_tool_info.velocity[j] = cv_pub[j];
    }
    wam_tool_pub.publish(wam_tool_info);

    force_msg.force[0] = staticForceEstimator.computedF[0];
    force_msg.force[1] = staticForceEstimator.computedF[1];
    force_msg.force[2] = staticForceEstimator.computedF[2];
    force_msg.force_norm = staticForceEstimator.computedF.norm(); //N in base frame 
    forceNorm = staticForceEstimator.computedF;
    forceNorm.normalize();
    force_msg.force_dir[0] = forceNorm[0];
    force_msg.force_dir[1] = forceNorm[1];
    force_msg.force_dir[2] = forceNorm[2];
    
    wam_estimated_contact_force_pub.publish(force_msg);
    
    
}

//wam_main Function
template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam)
{
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    ros::init(argc, argv, "wam");
    ros::NodeHandle n_;
    PlanarHybridControl<DOF> planar_hybrid_controller(wam, pm);
    planar_hybrid_controller.init(pm);
    ROS_INFO_STREAM("wam node initialized");
    ros::Rate pub_rate(PUBLISH_FREQ);

    while (ros::ok() && pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
        ros::spinOnce();
        planar_hybrid_controller.publish_wam(pm);
        pub_rate.sleep();
    }
    ROS_INFO_STREAM("wam node shutting down");
    return 0;
}
