/*
 * spacemouse_teleop_rviz.cpp
 *
 *  
 * 
 * Created on: Dec., 2023
 * Author: Faezeh
 */

#include "spacemouse_teleop_rviz.h"

void JoyToMovementPrimitives::init(){
    //initializing rviz params.

    a = 0.0;
    b = 0.0;
    p1 << 0.00, 0.00, 0.15;
    p2 << 0.00, 0.1, 0.15;
    p3 << 0.16, 0.2, 0.15;
    p4 << 0.0, 0.0, 0.0;
    R1 = Eigen::MatrixXd::Identity(2, 2);
    motion_dir = true;
    bases_changed = true;

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

    // Set the orientation using a quaternion
    
    quat.x = 0.0;
    quat.y = 0.0;
    quat.z = 0.0;  // For no rotation, set all quaternion values to 0
    quat.w = 1.0;  // This value represents no rotation as well

    pose1.pose.orientation = quat;
    pose2.pose.orientation = quat;
    pose3.pose.orientation = quat;

    poses.push_back(pose1);
    poses.push_back(pose2);
    poses.push_back(pose3);

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


    //Initializing publishers
    table_marker_pub_ = n_.advertise<visualization_msgs::Marker>("table_marker", 1, true);
    line_marker_pub_ = n_.advertise<visualization_msgs::Marker>("line_marker", 1, true);
    base_line_marker_pub_ = n_.advertise<visualization_msgs::Marker>("base_line_marker", 1, true);
    pose_marker_pub_ = n_.advertise<visualization_msgs::Marker>("pose_markers", 1, true);

    //Initializing subscribers
    joy_sub_ = n_.subscribe("/spacenav/joy", 1, &JoyToMovementPrimitives::joyCallback, this);

    drawTable(10,10);
    drawPoses();
    drawBasePoses();
    
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
        left button,
        right button,
    ]*/
    
    curr = msg->buttons;
    prev = prev_button_stats_;

    for (std::size_t i = 0; i < curr.size(); ++i) {
        pressedButtons[i] = (!prev[i] && curr[i]);  // True if the button is pressed in the current state but not in the previous state
    }
     
    prev_button_stats_ = curr;

    for (std::size_t i = 0; i < speed_multiplier_.size(); ++i) {
            speed_divider_[i] /= speed_multiplier_[i];
        }

    if (pressedButtons[0]) {
        scaleArray(speed_scale_, speed_divider_);
    } else if (pressedButtons[1]) {
        scaleArray(speed_scale_, speed_multiplier_);
    }
    
    R = findingRotationMatrix((p2-p1), (p3-p2));
    joy_axis << msg->axes[0], msg->axes[1], msg->axes[2];
    rotated_joy_axis = R * joy_axis.segment(0, 2);

    a += rotated_joy_axis[0] * speed_scale_[0];
    b += rotated_joy_axis[1] * speed_scale_[1]; 

    //or
    /*a = msg->axes[0] * speed_scale_[0];
    b = msg->axes[1] * speed_scale_[1];*/


    if (a > 1){a = 1;} else if (a < -1) {a = -1;}
    if (b > 1){b = 1;} else if (b < -1) {b = -1;}
    
    
    if(abs(msg->axes[0]) >= 0.01 || abs(msg->axes[1]) >= 0.01){
        if(motion_dir){p4 = p3 + (a/(p2-p1).norm())*(p2-p1) + (b/(p3-p2).norm())*(p3-p2);}
        else { p4 = p3 + (b/(p2-p1).norm())*(p2-p1) + (a/(p3-p2).norm())*(p3-p2);}
        // Add the received pose to the list
        geometry_msgs::PoseStamped new_pose;
        new_pose.pose.position.x = p4[0];
        new_pose.pose.position.y = p4[1];
        new_pose.pose.position.z = p4[2];
        new_pose.pose.orientation = quat;
        poses.push_back(new_pose); 
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
    
    // Publish the updated poses and connecting lines
    publishPoses();
    drawPoses();
    drawBasePoses();
    
    
       
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

Eigen::Matrix2d JoyToMovementPrimitives::findingRotationMatrix(const Eigen::Vector3d& vector1, const Eigen::Vector3d& vector2){
    x_axis << 1, 0, p1(2);
    y_axis << 0, 1, p1(2);
    x_naxis << -1, 0, p1(2);
    y_naxis << 0, -1, p1(2);
    
    theta_vec << angleBetweenVectors(x_axis,vector1) , angleBetweenVectors(x_axis,vector2), angleBetweenVectors(x_naxis,vector1), angleBetweenVectors(x_naxis,vector2);
    std::cout<<minAbsElement(theta_vec) + 1<<std::endl;
    switch (minAbsElement(theta_vec)){
        case 0:
            //v1 near x, v2 near y or-y
            motion_dir = true;
            ROS_INFO("1.");
            gamma_vec << angleBetweenVectors(y_axis,vector2) , angleBetweenVectors(y_naxis,vector2);
            
            if (minAbsElement(gamma_vec) == 1) {

                R1(0,0) = 1;
                R1(0,1) = 0;
                R1(1,0) = 0;
                R1(1,1) = -1;
                ROS_INFO("1n.");
            }
            alpha = (theta_vec(minAbsElement(theta_vec)) + gamma_vec(minAbsElement(gamma_vec)))*0.5;
            break;
        
        case 1:
            //v2 near x, v1 near y or -y
            motion_dir = false;
            ROS_INFO("2.");
            gamma_vec << angleBetweenVectors(y_axis,vector1) , angleBetweenVectors(y_naxis,vector1);
            
            if (minAbsElement(gamma_vec) == 1) {

                R1(0,0) = 1;
                R1(0,1) = 0;
                R1(1,0) = 0;
                R1(1,1) = -1;
                ROS_INFO("2n.");
            }
            alpha = (theta_vec(minAbsElement(theta_vec)) + gamma_vec(minAbsElement(gamma_vec)))*0.5;
            break;
        
        case 2:
            //v1 near -x, v2 near y or -y
            motion_dir = true;
            ROS_INFO("3.");
            gamma_vec << angleBetweenVectors(y_axis,vector2) , angleBetweenVectors(y_naxis,vector2);
            
            if (minAbsElement(gamma_vec) == 0){
                R1(0,0) = -1;
                R1(0,1) = 0;
                R1(1,0) = 0;
                R1(1,1) = 1;
                ROS_INFO("3n.");
            } else {
                R1(0,0) = -1;
                R1(0,1) = 0;
                R1(1,0) = 0;
                R1(1,1) = -1;
                ROS_INFO("3nn.");
            }
            alpha = (theta_vec(minAbsElement(theta_vec)) + gamma_vec(minAbsElement(gamma_vec)))*0.5;
            break;
        
        case 3:
            //v2 near -x 
            motion_dir = false;
            ROS_INFO("4.");
            gamma_vec << angleBetweenVectors(y_axis,vector1) , angleBetweenVectors(y_naxis,vector1);
            
            if (minAbsElement(gamma_vec) == 0){
                
                R1(0,0) = -1;
                R1(0,1) = 0;
                R1(1,0) = 0;
                R1(1,1) = 1;
                ROS_INFO("4n.");
            } else {
                
                R1(0,0) = -1;
                R1(0,1) = 0;
                R1(1,0) = 0;
                R1(1,1) = -1;
                ROS_INFO("4nn.");
            }
            alpha = (theta_vec(minAbsElement(theta_vec)) + gamma_vec(minAbsElement(gamma_vec)))*0.5;
            break;
    }
    R2(0,0) = cos(alpha);
    R2(0,1) = sin(alpha); //the signs should be reverse but for unknown reasons it seams like it works better this way!!!
    R2(1,0) = -sin(alpha);
    R2(1,1) = cos(alpha);
    
    R = R2*R1;
    

    return R;
}

int JoyToMovementPrimitives::minAbsElement(const Eigen::VectorXd& angleVector){
    // Calculate the minimum absolute element
    double minAbsElement = angleVector.array().abs().minCoeff();

    // Find the index of the minimum absolute element
    int minAbsIndex = -1;
    for (int i = 0; i < angleVector.size(); ++i) {
        if (std::abs(angleVector(i)) == minAbsElement) {
            minAbsIndex = i;
            break;
        }
    }

    return minAbsIndex;
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
    ROS_INFO("Table Published.");
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
    // Create a Marker message for connecting lines between poses
    visualization_msgs::Marker base_line_marker;
    base_line_marker.header.frame_id = "world";  // Change the frame_id according to your setup
    base_line_marker.header.stamp = ros::Time::now();
    base_line_marker.ns = "base_line_marker";
    base_line_marker.id = 1;
    base_line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    base_line_marker.action = visualization_msgs::Marker::ADD;
    base_line_marker.scale.x = 0.05;  // Line width

    // Set color
    base_line_marker.color.r = 1.0;
    base_line_marker.color.g = 0.0;
    base_line_marker.color.b = 1.0;
    base_line_marker.color.a = 1.0;

    // Add base vectors as red lines
    geometry_msgs::Point point1, point2, point3;
    point1.x = p1[0]+2;
    point1.y = p1[1]+2;
    point1.z = p1[2];

    point2.x = p2[0]+2;
    point2.y = p2[1]+2;
    point2.z = p2[2];

    point3.x = p3[0]+2;
    point3.y = p3[1]+2;
    point3.z = p3[2];

    base_line_marker.points.push_back(point1);
    base_line_marker.points.push_back(point2);
    base_line_marker.points.push_back(point3);

    // Publish the line marker
    base_line_marker_pub_.publish(base_line_marker);
    ROS_INFO("Lines Published.");
}

void JoyToMovementPrimitives::publishPoses() {
    // Create a Marker message for each pose
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";  // Change the frame_id according to your setup
    marker.header.stamp = ros::Time::now();
    marker.ns = "pose_markers";
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.005;
    marker.scale.y = 0.005;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    for (const auto& pose : poses) {
        marker.points.push_back(pose.pose.position);
    }

    // Publish the marker
    pose_marker_pub_.publish(marker);
    //ROS_INFO("Poses Published.");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "spacemouse_teleop_rviz");

    JoyToMovementPrimitives joy_teleop;

    joy_teleop.init();

    ros::spin();

    return 0;
}