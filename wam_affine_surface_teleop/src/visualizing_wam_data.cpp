#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <vector>

class PositionVisualizer {
private:
    ros::NodeHandle n_;
    ros::Publisher marker_pub_;
    ros::Subscriber pos_sub_desired_;
    ros::Subscriber pos_sub_current_;
    std::vector<geometry_msgs::Point> points_desired_;
    std::vector<geometry_msgs::Point> points_current_;
    bool first_point_received_ = false;
    double table_height_ = 0.05;  // Height of the table

public:
    PositionVisualizer() {
        marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        pos_sub_desired_ = n_.subscribe("/wam/new_pose", 10, &PositionVisualizer::callbackDesired, this);
        pos_sub_current_ = n_.subscribe("/wam/pose", 10, &PositionVisualizer::callbackCurrent, this);
    }

    void callbackDesired(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        points_desired_.push_back(msg->pose.position);
        publishLines(points_desired_, 1.0, 0.0, 0.0);  // Red lines
        if (!first_point_received_) {
            drawTable(msg->pose.position);
            first_point_received_ = true;
        }
    }

    void callbackCurrent(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        points_current_.push_back(msg->pose.position);
        publishLines(points_current_, 0.0, 1.0, 0.0);  // Green lines
    }

    void publishLines(const std::vector<geometry_msgs::Point>& points, float r, float g, float b) {
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "world";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "line_marker";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.scale.x = 0.01;
        line_marker.color.r = r;
        line_marker.color.g = g;
        line_marker.color.b = b;
        line_marker.color.a = 1.0;
        line_marker.points = points;
        marker_pub_.publish(line_marker);
    }

    void drawTable(const geometry_msgs::Point& first_point) {
        visualization_msgs::Marker table_marker;
        table_marker.header.frame_id = "world";
        table_marker.header.stamp = ros::Time::now();
        table_marker.ns = "table_marker";
        table_marker.id = 1;
        table_marker.type = visualization_msgs::Marker::CUBE;
        table_marker.action = visualization_msgs::Marker::ADD;
        table_marker.pose.position.x = first_point.x;
        table_marker.pose.position.y = first_point.y;
        table_marker.pose.position.z = first_point.z - table_height_ / 2;  // Center the table at z minus half its height
        table_marker.scale.x = 2.0;  // Arbitrary length and width
        table_marker.scale.y = 2.0;
        table_marker.scale.z = table_height_;
        table_marker.color.r = 0.5;
        table_marker.color.g = 0.5;
        table_marker.color.b = 0.5;
        table_marker.color.a = 1.0;
        marker_pub_.publish(table_marker);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "position_visualizer");
    PositionVisualizer visualizer;
    ros::spin();
    return 0;
}
