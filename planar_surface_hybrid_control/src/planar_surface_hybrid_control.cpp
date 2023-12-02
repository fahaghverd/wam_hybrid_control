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
void PlanarHybridControl<DOF>::init(ProductManager& pm){
    msg_timeout.fromSec(0.3);
    mypm = &pm;
    locked_joints = false;
    systems_connected = false;
  
    ROS_INFO("%zu-DOF WAM", DOF);
    jp_home = wam.getJointPositions();
    wam.gravityCompensate(true); // gravity compensation default set to true
    
    // setting up WAM joint state publisher
    const char* wam_jnts[] = {  
        "wam/YawJoint",
        "wam/ShoulderPitchJoint",
        "wam/ShoulderYawJoint",
        "wam/ElbowJoint",
        "wam/UpperWristYawJoint",
        "wam/UpperWristPitchJoint",
        "wam/LowerWristYawJoint"};
    std::vector < std::string > wam_joints(wam_jnts, wam_jnts + 7);
    wam_joint_state.name = wam_joints;
    wam_joint_state.name.resize(DOF);
    wam_joint_state.position.resize(DOF);
    wam_joint_state.velocity.resize(DOF);
    wam_joint_state.effort.resize(DOF);
    wam_jacobian_mn.data.resize(DOF*6);

    // ROS services
    go_home_srv = n_.advertiseService("go_home", &PlanarHybridControl::go_home_callback, this);
    joint_move_block_srv = n_.advertiseService("joint_move_block", &PlanarHybridControl::joint_move_block_callback, this);
    surface_calibrartion_srv = n_.advertiseService("surface_calibrartion", &PlanarHybridControl<DOF>::calibration, this);
    collect_cp_trajectory_srv = n_.advertiseService("collect_cp_trajectory", &PlanarHybridControl<DOF>::collectCpTrajectory, this);
    planar_surface_hybrid_control_srv = n_.advertiseService("planar_surface_hybrid_control", &PlanarHybridControl<DOF>::HybridCartImpForceCOntroller, this);
    disconnect_systems_srv = n_.advertiseService("disconnect_systems", &PlanarHybridControl::disconnectSystems, this);
    grid_test_calib_srv = n_.advertiseService("grid_test_calib", &PlanarHybridControl::grid_test_calibration, this);
    grid_test_srv = n_.advertiseService("grid_test", &PlanarHybridControl::grid_test, this);

    // ROS publishers
    wam_joint_state_pub = n_.advertise < sensor_msgs::JointState > ("joint_states", 1);
    wam_pose_pub = n_.advertise < geometry_msgs::PoseStamped > ("pose", 1);
    wam_jacobian_mn_pub = n_.advertise < wam_msgs::MatrixMN > ("jacobian",1);
    wam_tool_pub = n_.advertise < wam_msgs::RTToolInfo > ("tool_info",1);
    wam_estimated_contact_force_pub = n_.advertise < wam_msgs::RTCartForce > ("static_estimated_force",1);

    
    // ROS subscribers
    
    ROS_INFO("wam services now advertised");
    ros::AsyncSpinner spinner(0);
    spinner.start();
}

// Templated Surface Calibration Function
template<size_t DOF>
bool PlanarHybridControl<DOF>::calibration(wam_srvs::Teach::Request &req, wam_srvs::Teach::Response &res){   
    cp_type v1, v2, v3;
    systems::Ramp time(mypm->getExecutionManager());
    systems::TupleGrouper<double, cp_type, jp_type> configLogTg;
    const double T_s = mypm->getExecutionManager()->getPeriod();

    char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return false;
	}

    std::string path = "/home/wam/catkin_ws/src/wam_hybrid_control/.data/" + req.path;

    // Record at 1/10th of the loop rate
    systems::PeriodicDataLogger<config_sample_type> configLogger(
        mypm->getExecutionManager(),
        new barrett::log::RealTimeWriter<config_sample_type>(tmpFile, 10*T_s),
        10);

    std::cout<< "Move the robot to the first contact point and Press [Enter]."<<std::endl;
    waitForEnter();
    v1 = wam.getToolPosition();

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

    std::ofstream outputFile(path);

    if (!outputFile.is_open()) {
        printf("ERROR: Couldn't create the file!\n");
        return false;
    }
    log::Reader<config_sample_type> lr(tmpFile);
	lr.exportCSV(outputFile);
    std::remove(tmpFile);
    outputFile.close();

    surface_normal = ((v2-v1).cross(v3-v2));
    surface_normal.normalize();
    std::cout<<"Surface normal: "<< surface_normal << std::endl;   
    
    ROS_INFO_STREAM("Calibration finished. Press [Enter] to go home.");
    waitForEnter();
    go_home();

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

// Function to Collect Cartesian Trajectory
template <size_t DOF>
bool PlanarHybridControl<DOF>::collectCpTrajectory(wam_srvs::Teach::Request &req, wam_srvs::Teach::Response &res) {
    ROS_INFO("Collecting cartesian trajectory.");

    // Setup
    systems::Ramp time(mypm->getExecutionManager());
    systems::TupleGrouper<double, cp_type> cpLogTg;
    const double T_s = mypm->getExecutionManager()->getPeriod();

    // Create a temporary file for data storage
    char tmpFile[] = "/tmp/btXXXXXX";
    if (mkstemp(tmpFile) == -1) {
        perror("ERROR: Couldn't create temporary file!");
        return false;
    }

    // Set the file path for saving the trajectory data
    std::string path = "/home/wam/catkin_ws/src/wam_hybrid_control/.data/" + req.path;
    ROS_INFO_STREAM("Collecting cartesian trajectory. Saving to: " << path);

    // Record at 1/10th of the loop rate
    systems::PeriodicDataLogger<cp_sample_type> cpLogger(
        mypm->getExecutionManager(),
        new barrett::log::RealTimeWriter<cp_sample_type>(tmpFile, 10 * T_s),
        10);

    // Prompt to start collecting
    printf("Press [Enter] to start collecting.\n");
    waitForEnter();
    initial_point = wam.getToolPosition();

    {
        // Connect systems for data logging
        BARRETT_SCOPED_LOCK(mypm->getExecutionManager()->getMutex());
        connect(time.output, cpLogTg.template getInput<0>());
        connect(wam.toolPosition.output, cpLogTg.template getInput<1>());
        connect(cpLogTg.output, cpLogger.input);
        time.start();
    }

    // Prompt to stop collecting
    printf("Press [Enter] to stop collecting.\n");
    waitForEnter();

    // Finish data logging
    cpLogger.closeLog();
    disconnect(cpLogger.input);

    // Create and write trajectory data to the output file
    std::ofstream outputFile(path);
    log::Reader<cp_sample_type> lr(tmpFile);
    lr.exportCSV(outputFile);

    // Cleanup: Remove temporary file and close the output file
    std::remove(tmpFile);
    outputFile.close();

    // Finish the process
    ROS_INFO_STREAM("Collecting done. Press [Enter] to go home.");
    waitForEnter();
    go_home();

    return true;
}



// Hybrid force + impedance cp_position controller
//Lets play the collected trajectory as the trajectory! We cal also try collecting when not in contaact
//and then it projects. Also, we can reshape the trajectory based on the initial point and the projection
//matirx, like if its drawing z on the wall, should be able to draw it on the table as well.
template<size_t DOF>
bool PlanarHybridControl<DOF>::HybridCartImpForceCOntroller(wam_srvs::Play::Request &req, wam_srvs::Play::Response &res){
    //disconnectSystems();
    surface_normal[0] = 0.0;
    surface_normal[1] = 0.0;
    surface_normal[2] = -1.0;

    //Extracting cartesian trajectory from collected trajectory.
    std::string path = "/home/wam/catkin_ws/src/wam_hybrid_control/.data/" + req.path;
    std::ifstream inputFile(path);
    if (!inputFile.is_open()) {
        perror("ERROR: Couldn't open temporary file for reading!");
        return false;
    }
 
    // Read from the temporary file
    std::vector<cp_sample_type> vec;
    std::string line;
    while (std::getline(inputFile, line)) {
       std::istringstream iss(line);
       cp_sample_type sample;
       char comma;
       iss >> sample.get<0>() >> comma >> sample.get<1>().x() >> comma >> sample.get<1>().y() >> comma >> sample.get<1>().z();
       vec.push_back(sample);
    }

    // Extract cp_type values from vec
    std::vector<cp_type> cp_trj;
    for (const auto& record : vec) {
        cp_trj.push_back(boost::get<1>(record)); // Assuming cp_type is at index 1
    }
    initial_point = cp_trj[0];
    //initial_point[2] = initial_point[2]+0.015;

    //Force estimation
    // systems::connect(wam.kinematicsBase.kinOutput, getWAMJacobian.kinInput);
    // systems::connect(getWAMJacobian.output, staticForceEstimator.Jacobian);

    //systems::connect(wam.kinematicsBase.kinOutput, gravityTerm.kinInput);
    //systems::connect(gravityTerm.output, staticForceEstimator.g);

    //systems::connect(wam.jtSum.output, staticForceEstimator.jtInput);
   
    //Impedance Control
    jt_type jtLimits(35.0);
    cp_type cp_cmd, KpApplied, KdApplied;
    KpApplied << 300, 300, 300;
    KdApplied << 40, 30, 30;

    KxSet.setValue(KpApplied);
    DxSet.setValue(KdApplied);
    //XdSet.setValue(wam.getToolPosition());
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
    
    wam.idle();
    std::vector<units::CartesianPosition::type> waypoints = generateCubicSplineWaypoints(wam.getToolPosition(), initial_point, 0.01);
    std::cout<<"initial_point"<<initial_point<<std::endl;
    for (const auto& waypoint : waypoints) {
        // Move to the waypoint
        XdSet.setValue(waypoint);
        btsleep(0.5);
        cp_type e = (waypoint - wam.getToolPosition())/(waypoint.norm());
        if(e.norm() > 0.05) {std::cout<<e<<std::endl;}   
    }

    cp_type OrnKpApplied, OrnKdApplied;
    OrnKpApplied << 0, 0, 100;
    OrnKdApplied << 0, 0, 10;

    OrnKxSet.setValue(OrnKpApplied);
    OrnDxSet.setValue(OrnKdApplied);

    Eigen::Matrix3d Rotation = wam.getToolOrientation().toRotationMatrix();
    Rotation.col(2) << 0, 0, -1;

    // Ensure the resulting matrix is still a valid rotation matrix
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d Rotation_des = svd.matrixU() * svd.matrixV().transpose();

    // Check if the determinant is 1 to ensure it's a rotation matrix
    if (Rotation_des.determinant() < 0) {
        // If the determinant is -1, flip the sign of one of the singular vectors
        Eigen::Matrix3d flipMatrix = Eigen::Matrix3d::Identity();
        flipMatrix(2, 2) = -1;
        Rotation_des = svd.matrixU() * flipMatrix * svd.matrixV().transpose();
    }

    std::cout<<Rotation_des<<std::endl;
    Eigen::Quaterniond quaternion_des(Rotation_des);
    OrnXdSet.setValue(quaternion_des);

    // Convert Quaternion to Rotation Matrix
    cp_type euler_angles_d = Rotation_des.eulerAngles(2, 1, 0);

    int iterationCount = 0;
    cp_type projected_waypoint;
    for (int i = 40; i<cp_trj.size(); i+=5) {
        cp_type waypoint = cp_trj[i];

        std::cout<<i<<std::endl;
        // Calculate the vector from the point on the plane to the given point
        cp_type PQ;
        PQ = waypoint - initial_point;
    
        // Calculate the projection of PQ onto the plane's normal vector
        double projectionScalar;
        projectionScalar = PQ.dot(surface_normal) / surface_normal.squaredNorm();
        cp_type projection;
        projection = projectionScalar * surface_normal;

        // Move to the waypoint
        projected_waypoint = waypoint - projection;
        XdSet.setValue(projected_waypoint);
        btsleep(0.5);
        cp_type e = (projected_waypoint - wam.getToolPosition())/(projected_waypoint.norm());

        // Convert Quaternion to Rotation Matrix
        cp_type euler_angles = wam.getToolOrientation().toRotationMatrix().eulerAngles(2, 1, 0);

        cp_type Orne = (euler_angles_d - euler_angles)/(euler_angles_d.norm());
        //std::cout<< "waypoint:" << waypoint <<std::endl;
        //std::cout<< "projected_waypoint" << projected_waypoint <<std::endl;
        if(e.norm() > 0.05) {std::cout<<"position error: %"<<e*100<<std::endl;}   
        if(Orne.norm() > 0.05) {std::cout << "orientation error(zyx):"<<Orne* 180.0 / M_PI<< std::endl;
} 
    }

    systems::disconnect(torqueSum.output);

    return true;
}

// Function to generate waypoints along a Cubic Bezier curve and move to them
template <size_t DOF>
std::vector<units::CartesianPosition::type> PlanarHybridControl<DOF>::generateCubicSplineWaypoints(
    const units::CartesianPosition::type& initialPos,
    const units::CartesianPosition::type& finalPos,
    double offset
) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    std::vector<units::CartesianPosition::type> waypoints;

    Eigen::Vector3d deltaPos = finalPos - initialPos;
    double totalDistance = deltaPos.norm();

    // Calculate intermediate positions by offsetting x, y, and z
    Eigen::Vector3d yeeb = initialPos + deltaPos * offset;
    Eigen::Vector3d goalb = finalPos - deltaPos * offset;

    // Bezier interpolation
    int numPop = 100;
    std::vector<Eigen::Vector3d> plPop;
    for (int i = 0; i < numPop; ++i) {
        double t = static_cast<double>(i) / (numPop - 1);
        Eigen::Vector3d pl = (1 - t) * (1 - t) * (1 - t) * initialPos +
                             3 * (1 - t) * (1 - t) * t * yeeb +
                             3 * (1 - t) * t * t * goalb +
                             t * t * t * finalPos;
        plPop.push_back(pl);
    }

    // Calculate the subset of points to sample
    int steps = 20;
    double squish = 2.0;
    std::vector<double> midArr;
    for (double t = -squish; t <= squish; t += squish / (steps - 1)) {
        midArr.push_back(static_cast<double>(numPop) / (1 + std::exp(-t)));
    }

    double minMidArr = *std::min_element(midArr.begin(), midArr.end());
    double maxMidArr = *std::max_element(midArr.begin(), midArr.end());
    double m = (numPop - 1) / (maxMidArr - minMidArr);

    std::vector<int> iArr;
    for (double t : midArr) {
        int index = static_cast<int>(m * (t - minMidArr));
        iArr.push_back(index);
    }

    iArr[0] = 0;
    iArr.back() = numPop - 1;

    // List of points as a sample of bezier
    std::vector<units::CartesianPosition::type> pl;
    for (int index : iArr) {
        pl.push_back(cp_type(plPop[index]));
    }

    for (const auto& waypoint : pl) {
        waypoints.push_back(waypoint);
    }

    return waypoints;
}



template<size_t DOF>
void PlanarHybridControl<DOF>::disconnectSystems() {
    if (systems_connected) {
        systems::disconnect(wam.input);
        systems_connected = false;
        ROS_INFO("systems disconnected");
    } else {
        ROS_INFO("systems already disconnected");
    }
    return;
}

template<size_t DOF>
bool PlanarHybridControl<DOF>::disconnectSystems(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) { //??
    disconnectSystems();
    return true;
}

// goHome Function for sending the WAM safely back to its home starting position.
template<size_t DOF>
void PlanarHybridControl<DOF>::go_home()
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
    
    wam.idle();
    return;
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
{   //Current values to be published
    jp_type jp = wam.getJointPositions();
    jt_type jt = wam.getJointTorques();
    jv_type jv = wam.getJointVelocities();
    cp_type cp_pub = wam.getToolPosition();
    Eigen::Quaterniond to_pub = wam.getToolOrientation();
    math::Matrix<6,DOF> robot_tool_jacobian=wam.getToolJacobian();

    cv_type cv_pub = wam.getToolVelocity();
    cv_type cv_ang = wam.getToolAngularVelocity();

    //publishing sensor_msgs/JointState to wam/joint_states
    for (size_t i = 0; i < DOF; i++) {
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
    for (size_t h = 0; h < wam_jacobian_mn.n; ++h) {
        for (size_t k = 0; k < wam_jacobian_mn.m; ++k) {
            wam_jacobian_mn.data[h*6+k]=robot_tool_jacobian(k,h);
        }
    }
    wam_jacobian_mn_pub.publish(wam_jacobian_mn);

    //publish tool info to /wam/tool_info
    for (size_t j = 0; j < 3; j++) {
        wam_tool_info.position[j] = cp_pub[j];
        wam_tool_info.velocity[j] = cv_pub[j];
    }
    wam_tool_pub.publish(wam_tool_info);

    //publish static force estimation to /wam/static_estimated_force
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

template<size_t DOF>
bool PlanarHybridControl<DOF>::grid_test_calibration(wam_srvs::GridTestCalib::Request &req, wam_srvs::GridTestCalib::Response &res){
    systems::Ramp time(mypm->getExecutionManager());
    systems::TupleGrouper<double, cp_type, jp_type> configLogTg;
    const double T_s = mypm->getExecutionManager()->getPeriod();

    char tmpFile_w[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile_w) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return false;
	}

    std::string path_w = "/home/wam/catkin_ws/src/wam_hybrid_control/.data/gridTest/" + req.width_path;

    // Record at 1/10th of the loop rate
    systems::PeriodicDataLogger<config_sample_type> configLogger(
        mypm->getExecutionManager(),
        new barrett::log::RealTimeWriter<config_sample_type>(tmpFile_w, 10*T_s),
        10);

    
    std::cout<< "Move the robot to the first contact point and Press [Enter]."<<std::endl;
    waitForEnter();
    p1 = wam.getToolPosition();

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

    std::cout<< "Move the robot on the surface to the second point and Press [Enter]."<<std::endl;
    waitForEnter();

    time.stop();
    time.reset();
    configLogger.closeLog();
    disconnect(configLogger.input);

    std::ofstream outputFile_w(path_w);

    if (!outputFile_w.is_open()) {
        printf("ERROR: Couldn't create the file!\n");
        return false;
    }
    log::Reader<config_sample_type> lr_w(tmpFile_w);
	lr_w.exportCSV(outputFile_w);
    std::remove(tmpFile_w);
    outputFile_w.close();
    p2 = wam.getToolPosition();

    char tmpFile_l[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile_l) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return false;
	}
    // Record at 1/10th of the loop rate
    systems::PeriodicDataLogger<config_sample_type> configLogger_l(
        mypm->getExecutionManager(),
        new barrett::log::RealTimeWriter<config_sample_type>(tmpFile_l, 10*T_s),
        10);

    std::string path_l = "/home/wam/catkin_ws/src/wam_hybrid_control/.data/gridTest/" + req.length_path;
  
    std::cout<< "Press [Enter] and start collecting the lngth."<<std::endl;
    waitForEnter();
    {   
        // Make sure the Systems are connected on the same execution cycle
        // that the time is started. Otherwise we might record a bunch of
        // samples all having t=0; this is bad because the Spline requires time
        // to be monotonic.
        BARRETT_SCOPED_LOCK(mypm->getExecutionManager()->getMutex());
        connect(configLogTg.output, configLogger_l.input);
        time.start();
    }

    std::cout<< "Move the robot on the surface to the second point and Press [Enter]."<<std::endl;
    waitForEnter();

    configLogger_l.closeLog();
    disconnect(configLogger_l.input);

    std::ofstream outputFile_l(path_l);

    if (!outputFile_l.is_open()) {
        printf("ERROR: Couldn't create the file!\n");
        return false;
    }
    log::Reader<config_sample_type> lr_l(tmpFile_l);
	lr_l.exportCSV(outputFile_l);
    std::remove(tmpFile_l);
    outputFile_l.close();
    p3 = wam.getToolPosition();

    return true;
}

template<size_t DOF>
bool PlanarHybridControl<DOF>::grid_test(wam_srvs::GridTest::Request &req, wam_srvs::GridTest::Response &res){
    double a = req.a;
    double b = req.b;

    //V1 = p2-p1;
    //V2 = p3-p2;

    //cp_type cp_cmd = (a*V1+b*V2) + p1;
    //std::cout<< "p1:" << p1 <<std::endl;
    //std::cout<< "p2:" << p2 <<std::endl;
    //std::cout<< "p3:" << p3 <<std::endl;
    //std::cout<< "cp_cmd:" << cp_cmd <<std::endl;
    //cp_type cp_cmd;
    //cp_cmd[0] = 0.4988699631202508;//0.515487; 3.3%
    //cp_cmd[1] =  0.3713980418563303;//0.357516; 3.7%
    //cp_cmd[2] =  -0.2751059795701481;//-0.274274; 0.3%

    cp_type cp_cmd;
    cp_cmd[0] = 0.512028;//0.4988699631202508;// 2.6%
    cp_cmd[1] = 0.351117;// 0.3713980418563303; 5.4%
    cp_cmd[2] = -0.274203; //-0.2751059795701481;// 0.3%


    std::vector<units::CartesianPosition::type> waypoints = generateCubicSplineWaypoints(wam.getToolPosition(), cp_cmd, 0.0);

    for (const auto& waypoint : waypoints) {
        // Move to the waypoint
        wam.moveTo(waypoint);
        btsleep(0.5);
        cp_type e = (waypoint - wam.getToolPosition())/(waypoint.norm());
        if(e.norm() > 0.02) {std::cout<<e<<std::endl;}   
    }

    return true;

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
    
    // Moving to the start pose
    jp_type POS_READY;
    POS_READY << 0.002227924477643431, -0.1490540623980915, -0.04214558734519736, 1.6803055108189549;
    wam.moveTo(POS_READY);
    wam.idle();

    while (ros::ok() && pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
        ros::spinOnce();       
        planar_hybrid_controller.publish_wam(pm);
        pub_rate.sleep();
        
    }
    ROS_INFO_STREAM("wam node shutting down");
    return 0;
}
