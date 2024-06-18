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
#include <wam_msgs/RTCartVel.h>
#include <wam_msgs/RTOrtnVel.h>
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

static const float SPEED = 0.005;
static const int SPEED_MULTIPLIER = 2;
static const float SPEED_DIVIDER = 0.5;
const int CNTRL_FREQ = 50; // Frequency at which we will publish our control messages.

class JoyToMovementPrimitives{
    public:
        Eigen::Vector4d theta_vec;
        Eigen::Vector3d wamPos, p1, p2, p3, p4, x_axis, y_axis, x_naxis, y_naxis, joy_axis, a, b, v1, v2;
        Eigen::Vector2d rotated_joy_axis, gamma_vec;
        Eigen::Matrix2d R, R2, R1; //R1 for rotating x/y to -x/-y
        Eigen::Quaterniond wamOrt;
        // float a, b;
        double theta, gamma, alpha,betha;
        bool motion_dir, bases_changed, cart_publish, ortn_publish; //if true v1 near x, false v1 near y
        float cart_mag;
        std::vector<float> req_veldir;
        std::vector<double>  speed_scale_, speed_multiplier_, speed_divider_;
        std::vector<int> prev_button_stats_, pressedButtons, curr_button_stats_;
        geometry_msgs::Quaternion defaultQuat;

        //Messages
        visualization_msgs::Marker table_marker;
        std::vector<geometry_msgs::PoseStamped> poses;
        geometry_msgs::Point wam_pos;
        wam_msgs::RTCartVel cart_vel;
        wam_msgs::RTOrtnVel ortn_vel;

        // publishers
        ros::Publisher table_marker_pub_;
        ros::Publisher line_marker_pub_;
        ros::Publisher pose_marker_pub_;
        ros::Publisher wam_pos_marker_pub_;
        ros::Publisher base_line_marker_pub_;
        ros::Publisher cart_vel_pub;
        ros::Publisher ortn_vel_pub;

        // subscribed topics
        sensor_msgs::Joy space_joy_topic;

        // subscribers
        ros::Subscriber joy_sub_;
        ros::Subscriber wam_joint_states_sub_;
        ros::Subscriber wam_pose_sub_;

        //Service Clients
        ros::ServiceClient cart_imp_move_client;

    public:
        ros::NodeHandle n_;

        JoyToMovementPrimitives(){}

        ~JoyToMovementPrimitives(){}

        void init();
        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
        void drawTable(double length, double width);
        void drawPoses();
        void drawBasePoses();
        void publishPoses();
        double angleBetweenVectors(const Eigen::Vector3d& vector1, const Eigen::Vector3d& vector2);
        void scaleArray(std::vector<double>& array, const std::vector<double>& scale);
        Eigen::Vector3d projectVector(const Eigen::Vector3d& vecA, const Eigen::Vector3d& vecB);
        void wamPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void update();
        void drawWAMPoses();
};