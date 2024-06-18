/*
 * spacemouse_teleop_rviz.cpp
 *
 *  
 * 
 * Created on: Dec., 2023
 * Author: Faezeh
 */

#include "spacemouse_teleop_rviz+wam.h"

void JoyToMovementPrimitives::init(){
    //initializing rviz params.
    p1 << 0.00, 0.00, 0.15; // read these from wam!
    p2 << 0.2, 0.1, 0.15;
    // p4 << 0.0, 0.0, 0.0;

    cart_mag =0.05;

    // Set the orientation using a quaternion
    defaultQuat.x = 0.0;
    defaultQuat.y = 0.0;
    defaultQuat.z = 0.0;  // For no rotation, set all defaultQuaternion values to 0
    defaultQuat.w = 1.0;  // This value represents no rotation as well

    // Initialize three initial poses
    geometry_msgs::PoseStamped pose1, pose2, pose3;
    pose1.pose.position.x = p1[0];
    pose1.pose.position.y = p1[1];
    pose1.pose.position.z = p1[2];

    pose2.pose.position.x = p2[0];
    pose2.pose.position.y = p2[1];
    pose2.pose.position.z = p2[2];

    pose3.pose.position.x = p3[0];
    pose3.pose.position.y = p3[1];
    pose3.pose.position.z = p3[2];

    pose1.pose.orientation = defaultQuat;
    pose2.pose.orientation = defaultQuat;
    pose3.pose.orientation = defaultQuat;

    poses.push_back(pose1);
    poses.push_back(pose2);
    poses.push_back(pose3);

    speed_scale_ = {SPEED, SPEED};
    speed_multiplier_ = {SPEED_MULTIPLIER, SPEED_MULTIPLIER};
    speed_divider_ = {SPEED_DIVIDER, SPEED_DIVIDER};
    
    prev_button_stats_ = {0, 0};
    pressedButtons = {0, 0};
    
    cart_publish = ortn_publish = false; // Setting publisher states to false 
    
    //ROS Services
    // calibration_srv = n_.advertiseService("calibration", &JoyToMovementPrimitives::calibration, this);

    // //Service Clients
    // cart_imp_move_client = n_.serviceClient<wam_srvs::JointMoveBlock>("/wam/cp_impedance_control");

    //Initializing publishers
    table_marker_pub_ = n_.advertise<visualization_msgs::Marker>("table_marker", 1, true);
    line_marker_pub_ = n_.advertise<visualization_msgs::Marker>("line_marker", 1, true);
    base_line_marker_pub_ = n_.advertise<visualization_msgs::Marker>("base_line_marker", 1, true);
    wam_pos_marker_pub_ = n_.advertise<visualization_msgs::Marker>("wam_pose_markers", 1, true);
    // pose_marker_pub_ = n_.advertise<visualization_msgs::Marker>("pose_markers", 1, true);
    
    // WAM Publishers
    cart_vel_pub = n_.advertise<wam_msgs::RTCartVel>("cart_vel_cmd", 1);         // /wam/cart_vel_cmd
    ortn_vel_pub = n_.advertise<wam_msgs::RTOrtnVel>("ortn_vel_cmd", 1);         // /wam/ortn_vel_cmd

    //Initializing subscribers
    joy_sub_ = n_.subscribe("/spacenav/joy", 1, &JoyToMovementPrimitives::joyCallback, this);
    // wam_joint_states_sub_ = n_.subscribe("/wam/joint_states", 1, &JoyToMovementPrimitives::wamJointCallback, this);
    wam_pose_sub_ = n_.subscribe("/wam/pose", 1, &JoyToMovementPrimitives::wamPosCallback, this);

    p3 = wamPos;
    drawTable(50,50);
    drawBasePoses();   
}

void JoyToMovementPrimitives::wamPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    wamPos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    wamOrt.x() = msg->pose.orientation.x;
    wamOrt.y() = msg->pose.orientation.y;
    wamOrt.z() = msg->pose.orientation.z;
    wamOrt.w() = msg->pose.orientation.w;

    drawWAMPoses();
}

void JoyToMovementPrimitives::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
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
        left button,(
        right button,
    ]*/
    cart_publish = ortn_publish = false;

    p4 = wamPos;
    betha = angleBetweenVectors((p3-p2),(p4-p3));  

    if ((p4-p3).norm() >= 0.05  && abs(betha) >=0.7 && abs(betha) <= 2.44 && abs((p4-p3).norm() - (p3-p2).norm()) < 0.02){ // I should add a condtion to not change bases if we ARE moving in straight line!
        std::cout<<betha<<std::endl; 
        ROS_INFO("Bases Changed.");
        p1 = p2;
        p2 = p3;
        p3 = p4;
        a << 0.0,0.0,0.0;
        b << 0.0,0.0,0.0;
        //reset the speed as well
    }

    curr_button_stats_ = msg->buttons;

    for (std::size_t i = 0; i < curr_button_stats_.size(); ++i) {
        pressedButtons[i] = (!prev_button_stats_[i] && curr_button_stats_[i]);  // True if the button is pressed in the current state but not in the previous state
    }
     
    prev_button_stats_ = curr_button_stats_;

    if (pressedButtons[0]) {
        scaleArray(speed_scale_, speed_divider_);
    } else if (pressedButtons[1]) {
        scaleArray(speed_scale_, speed_multiplier_);
    }

    joy_axis << msg->axes[0], msg->axes[1], msg->axes[2];

    v1 = p2 - p1;
    v2 = p3 - p2;

    a += projectVector(joy_axis, v1) * speed_scale_[0];
    b += projectVector(joy_axis, v2) * speed_scale_[1];    
    

    if(abs(msg->axes[0]) >= 0.01 || abs(msg->axes[1]) >= 0.01){
        p4 =  p3 + a + b;

        req_veldir[0] = p4.normalized()[0];
        req_veldir[1] = p4.normalized()[1];
        req_veldir[2] = 0.0;
        cart_publish = true;

        // Add the received pose to the list
        geometry_msgs::PoseStamped new_pose;
        new_pose.pose.position.x = p4[0];
        new_pose.pose.position.y = p4[1];
        new_pose.pose.position.z = p4[2];
        new_pose.pose.orientation = defaultQuat;
        poses.push_back(new_pose); 

    } else {
        req_veldir = {0.0 , 0.0 , 0.0};
    }
    
    //Vel pub prepare

    // Publish the updated poses and connecting lines
    // publishPoses();
    drawPoses();
    drawBasePoses(); 
       
}

// Function to project vector A onto vector B
Eigen::Vector3d JoyToMovementPrimitives::projectVector(const Eigen::Vector3d& vecA, const Eigen::Vector3d& vecB) {
    double dotProd = vecA.dot(vecB);
    double magSquared = vecB.squaredNorm();
    double scalar = dotProd / magSquared;
    
    return scalar * vecB;
}

// Function to calculate the angle and direction between two vectors (from first to second vector)
double JoyToMovementPrimitives::angleBetweenVectors(const Eigen::Vector3d& vector1, const Eigen::Vector3d& vector2) {
    // Calculate the cross product
    Eigen::Vector3d crossProduct = vector1.cross(vector2);

    // Calculate the angle
    double angle = std::atan2(crossProduct.norm(), vector1.dot(vector2));

    // Determine the direction of the angle using the sign of the components of the cross product
    if (crossProduct(2) < 0) {
        angle = -angle;
    }

    return angle; //Radian
}

void JoyToMovementPrimitives::scaleArray(std::vector<double>& array, const std::vector<double>& scale) {
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

void JoyToMovementPrimitives::drawTable(double length, double width) {
    // Create a Marker message for the table
    table_marker.header.frame_id = "world"; // Change the frame_id according to your setup
    table_marker.header.stamp = ros::Time::now();
    table_marker.ns = "table_marker";
    table_marker.id = 0;
    table_marker.type = visualization_msgs::Marker::CUBE;
    table_marker.action = visualization_msgs::Marker::ADD;
    table_marker.pose.position.x = 0;
    table_marker.pose.position.y = 0;
    table_marker.pose.position.z = 0.0;
    table_marker.pose.orientation.w = 1.0;
    table_marker.scale.x = length;
    table_marker.scale.y = width;
    table_marker.scale.z = 0.3; // Height of the table

    // Set color
    table_marker.color.r = 0.0;
    table_marker.color.g = 1.0;
    table_marker.color.b = 0.0;
    table_marker.color.a = 1.0;

    // Publish the marker
    table_marker_pub_.publish(table_marker);
    // ROS_INFO("Table Published.");
}

void JoyToMovementPrimitives::drawPoses() {
    // Create a Marker message for connecting lines between poses
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = "world";  // Change the frame_id according to your setup
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "line_marker";
    line_marker.id = 1;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.scale.x = 0.01;  // Line width

    // Set color
    line_marker.color.r = 0.0;
    line_marker.color.g = 0.0;
    line_marker.color.b = 1.0;
    line_marker.color.a = 1.0;

    // Populate line_marker.points with initial poses
    for (const auto& pose : poses) {
        line_marker.points.push_back(pose.pose.position);
    }

    // Publish the line marker
    line_marker_pub_.publish(line_marker);
    //ROS_INFO("Lines Published.");
}

void JoyToMovementPrimitives::drawBasePoses() {
    // Create and publish multiple arrow markers to form a strip
    std::vector<geometry_msgs::Point> points;

    // Convert Eigen vectors to geometry_msgs::Point
    geometry_msgs::Point point1;
    point1.x = p1[0];
    point1.y = p1[1];
    point1.z = p1[2];
    points.push_back(point1);

    geometry_msgs::Point point2;
    point2.x = p2[0];
    point2.y = p2[1];
    point2.z = p2[2];
    points.push_back(point2);

    geometry_msgs::Point point3;
    point3.x = p3[0];
    point3.y = p3[1];
    point3.z = p3[2];
    points.push_back(point3);

    for (size_t i = 0; i < points.size() - 1; ++i) {
        visualization_msgs::Marker arrow_marker;
        arrow_marker.header.frame_id = "world";  // Change the frame_id according to your setup
        arrow_marker.header.stamp = ros::Time::now();
        arrow_marker.ns = "arrow_marker";
        arrow_marker.id = i;  // Each arrow needs a unique ID
        arrow_marker.type = visualization_msgs::Marker::ARROW;
        arrow_marker.action = visualization_msgs::Marker::ADD;

        // Set scale
        arrow_marker.scale.x = 0.005;  // Shaft diameter
        arrow_marker.scale.y = 0.01;   // Head diameter
        arrow_marker.scale.z = 0.01;   // Head length

        // Set color
        arrow_marker.color.r = 1.0;
        arrow_marker.color.g = 0.0;
        arrow_marker.color.b = 0.0;
        arrow_marker.color.a = 1.0;

        // Define start and end points of the arrow
        arrow_marker.points.push_back(points[i]);
        arrow_marker.points.push_back(points[i+1]);

        // Publish the arrow marker
        base_line_marker_pub_.publish(arrow_marker);
    }

    // ROS_INFO("Arrows Published.");
}

void JoyToMovementPrimitives::drawWAMPoses() {
    // Create a Marker message for connecting lines between poses
    visualization_msgs::Marker wam_pos_marker;
    wam_pos_marker.header.frame_id = "world";  
    wam_pos_marker.header.stamp = ros::Time::now();
    wam_pos_marker.ns = "wam_pos_markers";
    wam_pos_marker.id = 1;
    wam_pos_marker.type = visualization_msgs::Marker::POINTS;
    wam_pos_marker.action = visualization_msgs::Marker::ADD;
    wam_pos_marker.scale.x = 0.01;  
    wam_pos_marker.scale.y = 0.01;

    // Set color
    wam_pos_marker.color.r = 0.0;
    wam_pos_marker.color.g = 0.0;
    wam_pos_marker.color.b = 1.0;
    wam_pos_marker.color.a = 1.0;

    wam_pos.x = wamPos[0];
    wam_pos.y = wamPos[1];
    wam_pos.z = wamPos[2];
    wam_pos_marker.points.push_back(wam_pos);

    // Publish the line marker
    wam_pos_marker_pub_.publish(wam_pos_marker);
    //ROS_INFO("Lines Published.");
}

// Function for updating the commands and publishing
void JoyToMovementPrimitives::update()
{
    //Check our published cartesian velocity state and act accordingly
    if(cart_publish && !ortn_publish) // if only cart_publish state is set
    {
        cart_vel.direction[0] = req_veldir[0];
        cart_vel.direction[1] = req_veldir[1];
        cart_vel.direction[2] = req_veldir[2];
        cart_vel.magnitude = cart_mag;
        cart_vel_pub.publish(cart_vel);
    }
    
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "spacemouse_teleop_rviz");

    JoyToMovementPrimitives joy_teleop;
    joy_teleop.init();

    ros::Rate pub_rate(CNTRL_FREQ);
    while (joy_teleop.n_.ok()) // Looping at the specified frequency while our ros node is ok
    {
        ros::spinOnce();
        joy_teleop.update(); // Update and send commands to the WAM
        pub_rate.sleep();
    }

    return 0;
}