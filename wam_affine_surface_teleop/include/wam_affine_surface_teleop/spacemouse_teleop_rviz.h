/*
 * spacemouse_teleop_rviz.hpp
 *
 *  
 * 
 * Created on: Dec., 2023
 * Author: Faezeh
 */
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <string>
#include <sstream>
#include <fstream>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/Marker.h>
#include "ros/ros.h"

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <libconfig.h++>
#include <fcntl.h>
#include <termios.h>

class JoyToMovementPrimitives{
    public:
        Eigen::Vector4d theta_vec;
        Eigen::Vector3d p1, p2, p3, p4, x_axis, y_axis, x_naxis, y_naxis, joy_axis;
        Eigen::Vector2d rotated_joy_axis, gamma_vec;
        Eigen::Matrix2d R, R2, R1; //R1 for rotating x/y to -x/-y
        float a, b;
        double theta, gamma, alpha,betha;
        bool motion_dir, bases_changed; //if true v1 near x, false v1 near y

        std::vector<double>  speed_scale_, speed_multiplier_, speed_divider_;
        std::vector<int> prev_button_stats_, pressedButtons, curr, prev;
        geometry_msgs::Quaternion quat;

        // published topics
        visualization_msgs::Marker table_marker;
        std::vector<geometry_msgs::PoseStamped> poses;
        
        // publishers
        ros::Publisher table_marker_pub_;
        ros::Publisher line_marker_pub_;
        ros::Publisher pose_marker_pub_;
        ros::Publisher base_line_marker_pub_;

        // subscribed topics
        sensor_msgs::Joy space_joy_topic;

        // subscribers
        ros::Subscriber joy_sub_;

    public:
        ros::NodeHandle n_;

        JoyToMovementPrimitives(){}

        ~JoyToMovementPrimitives(){}

        void init();
        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
        void drawTable(double length, double width);
        void scaleArray(std::vector<double>& array, const std::vector<double>& scale);
        void drawPoses();
        void drawBasePoses();
        void publishPoses();
        double angleBetweenVectors(const Eigen::Vector3d& vector1, const Eigen::Vector3d& vector2);
        Eigen::Matrix2d findingRotationMatrix(const Eigen::Vector3d& vector1, const Eigen::Vector3d& vector2);
        int minAbsElement(const Eigen::VectorXd& angleVectors);
};