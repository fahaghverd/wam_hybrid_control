/*
 * spacemouse_teleop_wam.cpp
 *
 *  
 * 
 * Created on: Jan., 2024
 * Author: Faezeh
 */
#include "spacemouse_teleop_wam.h"

// Templated Initialization Function
template <size_t DOF>
void JoytoWAM<DOF>::init(ProductManager& pm)
{
    // Initialize member variables
    a = 0.0;
    b = 0.0;
    R1 = Eigen::MatrixXd::Identity(2, 2);

    bases_joy_corresponding = true;
    bases_changed = true;
    start_teleop = false;

    speed_scale_ = {0.001};
    ros::param::get("~initial_speed", speed_scale_);
    if (speed_scale_.size() == 1){
        speed_scale_ = std::vector<double>(2, speed_scale_[0]);
    } else if (speed_scale_.size() != 2){
        throw std::runtime_error("initial speed should be length of 1 or 2 but got" + std::to_string(speed_scale_.size()));
    }

    speed_multiplier_ = {2};
    speed_divider_ = {0.5, 0.5};
    ros::param::get("~initial_multiplier",speed_multiplier_);
    if (speed_multiplier_.size() == 1){
        speed_multiplier_ = std::vector<double>(2, speed_multiplier_[0]);
    } else if (speed_multiplier_.size() != 2){
        throw std::runtime_error("initial multiplier should be length of 1 or 2 but got" + std::to_string(speed_multiplier_.size()));
    }
    
    prev_button_stats_ = {0, 0};
    pressedButtons = {0, 0};

    msg_timeout.fromSec(0.3);
    mypm = &pm;
    locked_joints = false;
    systems_connected = false;
    force_estimated = false;
  
    // Log DOF information
    ROS_INFO("%zu-DOF WAM", DOF);

    // Get home joint positions and set up WAM
    jp_home = wam.getJointPositions();
    wam.gravityCompensate(true);
    pm.getSafetyModule()->setVelocityLimit(1.5);

    // Setting up WAM joint state publisher
    const char* wam_jnts[] = {
        "wam/YawJoint",
        "wam/ShoulderPitchJoint",
        "wam/ShoulderYawJoint",
        "wam/ElbowJoint",
        "wam/UpperWristYawJoint",
        "wam/UpperWristPitchJoint",
        "wam/LowerWristYawJoint"};

    std::vector<std::string> wam_joints(wam_jnts, wam_jnts + 7);
    wam_joint_state.name = wam_joints;
    wam_joint_state.name.resize(DOF);
    wam_joint_state.position.resize(DOF);
    wam_joint_state.velocity.resize(DOF);
    wam_joint_state.effort.resize(DOF);
    wam_jacobian_mn.data.resize(DOF * 6);

    // ROS services
    go_home_srv = n_.advertiseService("go_home", &JoytoWAM::goHomeCallback, this);
    joint_move_block_srv = n_.advertiseService("joint_move_block", &JoytoWAM::jointMoveBlockCallback, this);
    calibration_srv = n_.advertiseService("calibration", &JoytoWAM<DOF>::calibration, this);
    disconnect_systems_srv = n_.advertiseService("disconnect_systems", &JoytoWAM::disconnectSystems, this);
    contact_control_teleop_srv = n_.advertiseService("contact_control_teleop", &JoytoWAM::contactControlTeleop, this);

    // ROS publishers
    initPublisher<sensor_msgs::JointState>(wam_joint_state_pub, "joint_states", 1);
    initPublisher<geometry_msgs::PoseStamped>(wam_pose_pub, "pose", 1);
    initPublisher<wam_msgs::MatrixMN>(wam_jacobian_mn_pub, "jacobian", 1);
    initPublisher<wam_msgs::RTToolInfo>(wam_tool_pub, "tool_info", 1);
    initPublisher<wam_msgs::RTCartForce>(wam_estimated_contact_force_pub, "static_estimated_force", 1);

    // ROS subscribers
    joy_sub_ = n_.subscribe("/spacenav/joy", 1, &JoytoWAM::joyCallback, this);

    // Connect Spring System
    connectSystems();

    ROS_INFO("WAM services now advertised");
    ros::AsyncSpinner spinner(0);
    spinner.start();
}

template <size_t DOF>
template <typename T>
void JoytoWAM<DOF>::initPublisher(ros::Publisher& publisher, const std::string& topic_name, uint32_t queue_size)
{
    publisher = n_.advertise<T>(topic_name, queue_size);
}

template <size_t DOF>
void JoytoWAM<DOF>::connectSystems()
{
    // Add initial connections here
    systems::forceConnect(KxSet.output, ImpControl.KxInput);
    systems::forceConnect(DxSet.output, ImpControl.DxInput);
    systems::forceConnect(XdSet.output, ImpControl.XdInput);

    systems::forceConnect(OrnKxSet.output, ImpControl.OrnKpGains);
    systems::forceConnect(OrnDxSet.output, ImpControl.OrnKdGains);
    systems::forceConnect(OrnXdSet.output, ImpControl.OrnReferenceInput);

    systems::forceConnect(wam.toolPosition.output, ImpControl.CpInput);
    systems::forceConnect(wam.toolVelocity.output, ImpControl.CvInput);
    systems::forceConnect(wam.toolOrientation.output, ImpControl.OrnInput);

    systems::forceConnect(wam.kinematicsBase.kinOutput, ImpControl.kinInput);
    systems::forceConnect(wam.kinematicsBase.kinOutput, toolforce2jt.kinInput);
    systems::forceConnect(wam.kinematicsBase.kinOutput, tt2jt_ortn_split.kinInput);

    systems::forceConnect(ImpControl.CFOutput, toolforce2jt.input);
    systems::forceConnect(ImpControl.CTOutput, tt2jt_ortn_split.input);
    systems::forceConnect(FeedFwdForce.output, toolforcefeedfwd2jt.input);
}

// Templated Surface Calibration Function
template<size_t DOF>
bool JoytoWAM<DOF>::calibration(wam_srvs::Teach::Request &req, wam_srvs::Teach::Response &res) {
    // Define constants and systems
    const double T_s = mypm->getExecutionManager()->getPeriod();
    const int loggingRateMultiplier = 10;
    systems::Ramp time(mypm->getExecutionManager());
    systems::TupleGrouper<double, cp_type, jp_type> configLogTg;

    // Create a temporary file for logging
    char tmpFile[] = "/tmp/btXXXXXX";
    if (mkstemp(tmpFile) == -1) {
        ROS_ERROR("Couldn't create temporary file!");
        return false;
    }

    // Define file paths
    std::string path_trj = "/home/wam/catkin_ws/src/wam_hybrid_control/.data/joyToWamCalib/" + req.path + "Trj";
    std::string path_pnts = "/home/wam/catkin_ws/src/wam_hybrid_control/.data/" + req.path + "Pts";

    // Record at 1/10th of the loop rate
    systems::PeriodicDataLogger<config_sample_type> configLogger(
        mypm->getExecutionManager(),
        new barrett::log::RealTimeWriter<config_sample_type>(tmpFile, loggingRateMultiplier * T_s),
        loggingRateMultiplier);

    // Record the first point
    std::cout << "Move the robot to the first contact point and press [Enter]." << std::endl;
    waitForEnter();
    p1 = wam.getToolPosition();
    P1 = wam.getJointPositions();
    pts.col(0) = p1;

    {
        // Make sure the Systems are connected on the same execution cycle
        // that the time is started. Otherwise we might record a bunch of
        // samples all having t=0; this is bad because the Spline requires time
        // to be monotonic.
        // Connect systems on the same execution cycle as starting time
        BARRETT_SCOPED_LOCK(mypm->getExecutionManager()->getMutex());
        connect(time.output, configLogTg.template getInput<0>());
        connect(wam.toolPosition.output, configLogTg.template getInput<1>());
        connect(wam.jpOutput, configLogTg.template getInput<2>());
        connect(configLogTg.output, configLogger.input);
        time.start();
    }

    // Record the second point
    std::cout << "Move the robot on the surface to the second point and press [Enter]." << std::endl;
    waitForEnter();
    p2 = wam.getToolPosition();
    P2 = wam.getJointPositions();
    pts.col(1) = p2;

    // Record the third point
    std::cout << "Move the robot onto the third point and press [Enter]." << std::endl;
    waitForEnter();
    p3 = wam.getToolPosition();
    P3 = wam.getJointPositions();
    pts.col(2) = p3;

    // Close the logger
    configLogger.closeLog();
    disconnect(configLogger.input);

    std::ofstream outputFile(path_trj);
    if (!outputFile.is_open()) {
        printf("ERROR: Couldn't create the file!\n");
        return false;
    }

    // Export data to CSV file
    log::Reader<config_sample_type> lr(tmpFile);
	lr.exportCSV(outputFile);
    std::remove(tmpFile);

    // Write the matrix to the CSV file
    std::ofstream outputFile2(path_pnts);
    if (!outputFile2.is_open()) {
        ROS_ERROR("Couldn't create the file!");
        return false;
    }

    for (int i = 0; i < pts.rows(); ++i) {
        for (int j = 0; j < pts.cols(); ++j) {
            outputFile2 << pts(i, j);
            if (j < pts.cols() - 1) {
                outputFile2 << ",";
            }
        }
        outputFile2 << std::endl;
    }
    outputFile2.close();

    // Compute surface normal
    surface_normal = ((p2 - p1).cross(p3 - p2)).normalized();
    ROS_INFO_STREAM("Surface normal: " << surface_normal);

    ROS_INFO("Calibration finished. Press [Enter] to go home.");
    waitForEnter();
    goHome();

    return true;
}

template<size_t DOF>
void JoytoWAM<DOF>::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    /*:type msg: sensor_msgs.msg.Joy

    axes: [
        x (front)
        y (left)
        z (up)
        R (around x)
        P (around y)
        Y (around z)
    ]
    buttons: [
        left button,
        right button,
    ]*/
    if(start_teleop){
        updateButtonStatus(msg->buttons);

        for (std::size_t i = 0; i < speed_multiplier_.size(); ++i) {
                speed_divider_[i] /= speed_multiplier_[i];
            }
    
        adjustSpeedScale();
        
        R = findingRotationMatrix((p2-p1), (p3-p2));
        joy_axis << msg->axes[0], msg->axes[1];
        rotated_joy_axis = R * joy_axis.segment(0, 2);

        a += rotated_joy_axis[0] * speed_scale_[0];
        b += rotated_joy_axis[1] * speed_scale_[1]; 

        //or
        /*a = msg->axes[0] * speed_scale_[0];
        b = msg->axes[1] * speed_scale_[1];*/

        limitValues(a, -1, 1);
        limitValues(b, -1, 1);
        
        if(abs(msg->axes[0]) >= 0.01 || abs(msg->axes[1]) >= 0.01){
            if(bases_joy_corresponding){
                p4 = p3 + (a/(p2-p1).norm())*(p2-p1) + (b/(p3-p2).norm())*(p3-p2);
            } else { 
                p4 = p3 + (b/(p2-p1).norm())*(p2-p1) + (a/(p3-p2).norm())*(p3-p2);
            }

            // Add the received pose to the list
            geometry_msgs::PoseStamped new_pose;
            new_pose.pose.position.x = p4[0];
            new_pose.pose.position.y = p4[1];
            new_pose.pose.position.z = p4[2];

            betha = angleBetweenVectors((p3-p2),(p4-p3)); 

            if ((p4-p3).norm() >= 0.2  && abs(betha) >=0.5 && abs(betha) <= 2.62) {
                std::cout<<betha<<std::endl; 
                ROS_INFO("Bases Changed.");
                p1 = p2;
                p2 = p3;
                p3 = p4;
                a = 0;
                b = 0;
                //reset the speed as well
            }
        }     
    }
}

// Helper Function to Update Button Status
template<size_t DOF>
void JoytoWAM<DOF>::updateButtonStatus(const std::vector<int>& buttons) {
    curr_button_stats_ = buttons;

    for (std::size_t i = 0; i < curr_button_stats_.size(); ++i) {
        pressedButtons[i] = (!prev_button_stats_[i] && curr_button_stats_[i]);  // True if the button is pressed in the current state but not in the previous state
    }

    prev_button_stats_ = curr_button_stats_;
}

template<size_t DOF>
void JoytoWAM<DOF>::scaleArray(std::vector<double>& array, const std::vector<double>& scale) {
    // Check if the sizes of the input vector and the scaling factor vector match
    if (array.size() == scale.size()) {
        // Scale each element of the array by the corresponding element in the scale vector
        for (std::size_t i = 0; i < array.size(); ++i) {
            array[i] *= scale[i];
        }
    } else {
        throw std::runtime_error("Scaled arrays should have same size.");
    }
}

// Helper Function to Adjust Speed Scale
template<size_t DOF>
void JoytoWAM<DOF>::adjustSpeedScale() {
    for (std::size_t i = 0; i < speed_scale_.size(); ++i) {
        speed_scale_[i] = (pressedButtons[0]) ? (speed_scale_[i] / speed_divider_[i]) : (speed_scale_[i] * speed_multiplier_[i]);
    }
}

// Helper Function to Limit Values
template<size_t DOF>
void JoytoWAM<DOF>::limitValues(float& value, double min, double max) {
    if (value > max) {
        value = max;
    } else if (value < min) {
        value = min;
    }
}

template<size_t DOF>
Eigen::Matrix2d JoytoWAM<DOF>::findingRotationMatrix(const Eigen::Vector3d& vector1, const Eigen::Vector3d& vector2){
    // Initialize coordinate axes
    x_axis << 1, 0, p1(2);
    y_axis << 0, 1, p1(2);
    x_naxis << -1, 0, p1(2);
    y_naxis << 0, -1, p1(2);
    
    // Calculate angles
    theta_vec << angleBetweenVectors(x_axis,vector1) , angleBetweenVectors(x_axis,vector2), 
                 angleBetweenVectors(x_naxis,vector1), angleBetweenVectors(x_naxis,vector2);

    
    // Determine base and rotation
    int thetaMinIndex = minAbsElement(theta_vec);
    bases_joy_corresponding = (thetaMinIndex == 0 || thetaMinIndex == 2);

    switch (thetaMinIndex) {
        case 0: // v1 near x, v2 near y or -y
            ROS_INFO("1.");
            gamma_vec << angleBetweenVectors(y_axis, vector2), angleBetweenVectors(y_naxis, vector2);
            if (minAbsElement(gamma_vec) == 1) {
                R1 << 1, 0, 0, -1;
                ROS_INFO("n.");
            }
            break;

        case 1: // v2 near x, v1 near y or -y
            ROS_INFO("2.");
            gamma_vec << angleBetweenVectors(y_axis, vector1), angleBetweenVectors(y_naxis, vector1);
            if (minAbsElement(gamma_vec) == 1) {
                R1 << 1, 0, 0, -1;
                ROS_INFO("n.");
            }
            break;

        case 2: // v1 near -x, v2 near y or -y
            ROS_INFO("3.");
            gamma_vec << angleBetweenVectors(y_axis, vector2), angleBetweenVectors(y_naxis, vector2);
            if (minAbsElement(gamma_vec) == 0) {
                R1 << -1, 0, 0, 1;
                ROS_INFO("n.");
            } else {
                R1 << -1, 0, 0, -1;
                ROS_INFO("nn.");
            }
            break;

        case 3: // v2 near -x
            ROS_INFO("4.");
            gamma_vec << angleBetweenVectors(y_axis, vector1), angleBetweenVectors(y_naxis, vector1);
            if (minAbsElement(gamma_vec) == 0) {
                R1 << -1, 0, 0, 1;
                ROS_INFO("n.");
            } else {
                R1 << -1, 0, 0, -1;
                ROS_INFO("nn.");
            }
            break;
    }

    alpha = (theta_vec(thetaMinIndex) + gamma_vec(minAbsElement(gamma_vec))) * 0.5;
    R2 << cos(alpha), sin(alpha),
          -sin(alpha), cos(alpha); //I think sin signs should be reverse.
    
    R = R2*R1;
    

    return R;
}

template<size_t DOF>
int JoytoWAM<DOF>::minAbsElement(const Eigen::VectorXd& angleVector){
    double minAbsElement = angleVector.array().abs().minCoeff();

    int minAbsIndex = -1;
    for (int i = 0; i < angleVector.size(); ++i) {
        if (std::abs(angleVector(i)) == minAbsElement) {
            minAbsIndex = i; // what if twi elemnt have the min: not possible?
            break;
        }
    }

    return minAbsIndex;
}

template<size_t DOF>
// Function to calculate the angle and direction between two vectors (from first to second vector)
double JoytoWAM<DOF>::angleBetweenVectors(const Eigen::Vector3d& vector1, const Eigen::Vector3d& vector2) {
    Eigen::Vector3d crossProduct = vector1.cross(vector2);
    double angle = std::atan2(crossProduct.norm(), vector1.dot(vector2));

    if (crossProduct(2) < 0) {
        angle = -angle;
    }

    return angle; //Radian
}

template<size_t DOF>
bool JoytoWAM<DOF>::contactControlTeleop(wam_srvs::ContactControlTeleop::Request &req, wam_srvs::ContactControlTeleop::Response &res){
    
    if(req.start){
        //Extracting cartesian points from collected trajectory in calibration.
        std::string path_trj = "/home/wam/catkin_ws/src/wam_hybrid_control/.data/joyToWamCalib/" + req.path + "Trj";
        std::string path_pnts = "/home/wam/catkin_ws/src/wam_hybrid_control/.data/" + req.path + "Pts";
        std::ifstream inputFile(path_trj);
        if (!inputFile.is_open()) {
            perror("ERROR: Couldn't open temporary file for reading!");
        }

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
            cp_trj.push_back(boost::get<1>(record));
        }

        std::ifstream inputFile2(path_pnts);
        if (!inputFile2.is_open()) {
            perror("ERROR: Couldn't open temporary file for reading!");
        }

        for (int i = 0; i < pts.rows(); ++i) {
            for (int j = 0; j < pts.cols(); ++j) {
                // Read the element from the file
                if (!(inputFile2 >> pts(i, j))) {
                    std::cerr << "Error reading from file: " << path_pnts << std::endl;
                }

                // Check for a comma (skip it if present)
                if (j < pts.cols() - 1) {
                    char comma;
                    inputFile2 >> comma;
                    if (comma != ',') {
                        std::cerr << "Error: Expected comma in file: " << path_pnts << std::endl;
                    }
                }
            }
        }
        p1 = pts.col(0);
        p2 = pts.col(1);
        p3 = pts.col(2);

        std::cout<< "Press [Enter] to move to the initial point on the table."<<std::endl;
        waitForEnter();
        
        std::vector<cp_type>  p1_trj = generateCubicSplineWaypoints(wam.getToolPosition(), p1, 0.0);
        //Impedance Control params
        cp_type KpApplied, KdApplied;
        KpApplied << 100, 100, 100;
        KdApplied << 20, 20, 20;
        CartImpController(p1_trj, 1, KpApplied, KdApplied);

        std::cout<< "Press [Enter] to move to replay the base vectors."<<std::endl;
        waitForEnter();
        CartImpController(cp_trj, 1, KpApplied, KdApplied);

        std::cout<< "Press [Enter] to start navigating the WAM on the surface with SpaceMouse."<<std::endl;
        waitForEnter();
        start_teleop = true;

        return true;

    } else {
        start_teleop = false;
        a = 0;
        b = 0;

        return true;
    } 
}

template<size_t DOF>
void JoytoWAM<DOF>::CartImpController(std::vector<cp_type> &Trajectory, int step, const cp_type &KpApplied, const cp_type &KdApplied,
                                                 bool orientation_control, const cp_type &OrnKpApplied, const cp_type &OrnKdApplied,
                                                 bool ext_force, const cf_type &des_force, bool null_space){
    //Impedance Control params
    KxSet.setValue(KpApplied);
    DxSet.setValue(KdApplied);
    OrnKxSet.setValue(OrnKpApplied);
    OrnDxSet.setValue(OrnKdApplied);
    FeedFwdForce.setValue(des_force);

    // CONNECT TO SUMMER
    systems::forceConnect(toolforce2jt.output, torqueSum.getInput(0));
    systems::forceConnect(tt2jt_ortn_split.output, torqueSum.getInput(1));
    if(ext_force){systems::forceConnect(toolforcefeedfwd2jt.output, torqueSum.getInput(2));}

    /* tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) - this->jacobian_.transpose() * jacobian_transpose_pinv) *
                         (this->nullspace_stiffness_ * (this->q_d_nullspace_ - this->q_) - this->nullspace_damping_ * this->dq_);*/

    // SATURATE AND CONNECT TO WAM INPUT
    systems::forceConnect(torqueSum.output, jtSat.input);        
    systems::forceConnect(jtSat.output, wam.input); 

    cp_type waypoint;
    if(orientation_control){
        Eigen::Matrix3d Rotation;
        for (int i = 0; i<Trajectory.size(); i+=step) {
            // Find the desired rotation
            Rotation = wam.getToolOrientation().toRotationMatrix();
            Rotation.col(2) << surface_normal;
            
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

            // Convert Quaternion to Rotation Matrix
            std::cout << "Rotation_des:" << Rotation_des << std::endl;
            Eigen::Quaterniond quaternion_des(Rotation_des);
            OrnXdSet.setValue(quaternion_des);
            
            // Move to the waypoint
            waypoint = Trajectory[i];
            std::cout<<i<<std::endl;
            XdSet.setValue(waypoint);
            btsleep(0.5);

            cp_type e = (waypoint - wam.getToolPosition())/(waypoint.norm());
            
            cp_type euler_angles = wam.getToolOrientation().toRotationMatrix().eulerAngles(2, 1, 0);
            cp_type euler_angles_d = Rotation_des.eulerAngles(2, 1, 0);
            cp_type Orne = (euler_angles_d - euler_angles)/(euler_angles_d.norm());
            
            if(e.norm() > 0.03) {std::cout<<"position error: %"<<e*100<<std::endl;}   
            if(Orne.norm() > 0.03) {std::cout << "orientation error(zyx):"<<Orne* 180.0 / M_PI<< std::endl;}
        }
    }
    else{
        OrnXdSet.setValue(wam.getToolOrientation());
        for (int i = 0; i<Trajectory.size(); i+=step) {
            // Move to the waypoint
            waypoint = Trajectory[i];
            XdSet.setValue(waypoint);
            btsleep(0.3);
            cp_type e = (waypoint - wam.getToolPosition())/(waypoint.norm());
            if(e.norm() > 0.03) {std::cout<<e<<std::endl;}   
        }
    }
    systems::disconnect(torqueSum.output);    
}

// Function to generate waypoints along a Cubic Bezier curve and move to them
template <size_t DOF>
std::vector<units::CartesianPosition::type> JoytoWAM<DOF>::generateCubicSplineWaypoints(
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
void JoytoWAM<DOF>::disconnectSystems() {
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
bool JoytoWAM<DOF>::disconnectSystems(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) { //??
    disconnectSystems();
    return true;
}

// goHome Function for sending the WAM safely back to its home starting position.
template<size_t DOF>
void JoytoWAM<DOF>::goHome()
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
bool JoytoWAM<DOF>::goHomeCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
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
bool JoytoWAM<DOF>::jointMoveBlockCallback(wam_srvs::JointMoveBlock::Request &req, wam_srvs::JointMoveBlock::Response &res)
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
void JoytoWAM<DOF>::publishWam(ProductManager& pm)
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
    if(staticForceEstimator.computedF.norm() > 14.0){ROS_INFO("Contact detected.");} 
    if(staticForceEstimator.computedF.norm() > 0.0){
        force_norm = staticForceEstimator.computedF;
        force_norm.normalize();
        force_msg.force_dir[0] = force_norm[0];
        force_msg.force_dir[1] = force_norm[1];
        force_msg.force_dir[2] = force_norm[2];
    }
    wam_estimated_contact_force_pub.publish(force_msg);

}

template <size_t DOF>
typename units::JointTorques<DOF>::type saturateJt(const typename units::JointTorques<DOF>::type& x,
                                                   const typename units::JointTorques<DOF>::type& limit) {
    int index;
    double minRatio;

    minRatio = limit.cwiseQuotient(x.cwiseAbs()).minCoeff(&index);

    if (minRatio < 1.0) {
        return minRatio * x;
    } else {
        return x;
    }
};

//wam_main Function
template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam)
{
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    ros::init(argc, argv, "wam");
    ros::NodeHandle n_;
    JoytoWAM<DOF> joy_to_wam(wam, pm);
    joy_to_wam.init(pm);
    ROS_INFO_STREAM("wam node initialized");
    ros::Rate pub_rate(PUBLISH_FREQ);

    // Moving to the start pose
    jp_type POS_READY;
    POS_READY << 0.002227924477643431, -0.1490540623980915, -0.04214558734519736, 1.6803055108189549;
    wam.moveTo(POS_READY);
    wam.idle();

    while (ros::ok() && pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
        ros::spinOnce();
        joy_to_wam.publishWam(pm);
        //joy_to_wam.updateRT(pm);
        pub_rate.sleep();
    }
    ROS_INFO_STREAM("wam node shutting down");
    return 0;
}

