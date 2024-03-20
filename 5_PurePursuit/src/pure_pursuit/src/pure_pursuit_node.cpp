#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <fstream>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

/// CHECK: include needed ROS msg type headers and libraries

using namespace std;
using std::placeholders::_1;
using namespace std::chrono_literals;

class PurePursuit : public rclcpp::Node
{
    // Implement PurePursuit

public:
    PurePursuit() : Node("pure_pursuit_node")
    {
       //constructor
       // load in data (path to track)
       std::string filePath = "/sim_ws/src/pure_pursuit/data/robot_log.csv";
       loadLogData(filePath);
       
       // Topics
        std::string lidarscan_topic = "/scan";
        std::string drive_topic = "/drive";
        std::string odometry_topic = "/ego_racecar/odom";

        // ROS subscribers and publishers
        drivepub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);

        std::cout << "--- Now Let's Make it Perty.... :) " << std::endl;
        viz_timer_ = this->create_wall_timer(10s, std::bind(&PurePursuit::drawLogData, this));

        markerpub = this->create_publisher<visualization_msgs::msg::Marker>("/visualize_goal",10);
        pathpub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualize_path",10);
        // lasersub = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 10, std::bind(&ReactiveFollowGap::scan_callback, this, _1));
        odomsub = this->create_subscription<nav_msgs::msg::Odometry>(odometry_topic, 10, std::bind(&PurePursuit::pose_callback, this, _1));
    
    }
    

private:

    //TUNABLE PARAMETERS
    double L = 1.5; //lookahead distance
    double PGain = 0.3;
    double velocity = 2;
    int step_size = 10; //step size for waypoint selection
    double max_steer = 3.14/2;
    double min_steer = -3.14/2;

    //trajectory being followed
    std::vector<double> x_traj;
    std::vector<double> y_traj;
    std::vector<double> yaw_traj;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drivepub;
    rclcpp::TimerBase::SharedPtr viz_timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerpub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pathpub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomsub;
    visualization_msgs::msg::Marker select_waypoint = visualization_msgs::msg::Marker();
    visualization_msgs::msg::MarkerArray path = visualization_msgs::msg::MarkerArray();


    void loadLogData(const std::string filePath){
        // read in file
        std::ifstream file(filePath);

        if(!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open log file: %s", filePath.c_str());
            return;
        }

        std::string row;
        while(std::getline(file, row)) {
            std::stringstream ss(row);

            // deliminater ','
            std::string value;
            if (getline(ss, value, ',')) {
                x_traj.push_back(std::stod(value));
            }
            if (getline(ss, value, ',')) {
                y_traj.push_back(std::stod(value));
            }
            if (getline(ss, value, ',')) {
                yaw_traj.push_back(std::stod(value));
            }
        }
        file.close();
        std::cout << "--- Sucessfully loaded data from log " << std::endl;
        // for(size_t i = 0; i<x_traj.size(); i++){
        //     std::cout << yaw_traj[int(i)] << std::endl;
        // }
    }

    void drawLogData(){
        for(size_t i=0; i<x_traj.size(); i+=step_size){
            visualization_msgs::msg::Marker path_point;
            path_point.header.frame_id = "map";
            path_point.header.stamp = this->get_clock()->now();
            path_point.ns = "path";
            path_point.id = i;
            path_point.type = visualization_msgs::msg::Marker::SPHERE;
            path_point.action = visualization_msgs::msg::Marker::ADD;
            path_point.pose.position.x = x_traj[i];
            path_point.pose.position.y = y_traj[i];
            path_point.pose.position.z = 0;
            path_point.pose.orientation.x = 0.0;
            path_point.pose.orientation.y = 0.0;
            path_point.pose.orientation.z = 0.0;
            path_point.pose.orientation.w = 1.0;
            path_point.scale.x = 0.05;
            path_point.scale.y = 0.05;
            path_point.scale.z = 0.05;
            path_point.color.a = 1.0;
            path_point.color.r = 0.0;
            path_point.color.g = 0.0;
            path_point.color.b = 1.0;

            path.markers.push_back(path_point);
        }
        pathpub->publish(path);
    }

    std::vector<double> select_goal(const geometry_msgs::msg::Pose pose_curr){
        int goal_index;
        size_t closest_index;
        double curr_x = pose_curr.position.x;
        double curr_y = pose_curr.position.y;
        double min_dist = 1000;
        double dist;

        //find closest point
        for(size_t i=0; i<x_traj.size(); i+=step_size){
            dist = sqrt(pow((x_traj[i]-curr_x),2) + pow((y_traj[i]-curr_y),2));
            if (dist < min_dist) {
                closest_index = i;
                min_dist = dist;
            }
        }

        //ensure lookahead distance enforced
        for(size_t i=closest_index; i<x_traj.size(); i++){
            dist = sqrt(pow((x_traj[i]-curr_x),2) + pow((y_traj[i]-curr_y),2));
            if (dist > L) {
                goal_index = int(i);
                break;
            }
        }

        select_waypoint.header.frame_id = "map";
        select_waypoint.header.stamp = this->get_clock()->now();
        select_waypoint.ns = "goal";
        select_waypoint.type = visualization_msgs::msg::Marker::SPHERE;
        select_waypoint.action = visualization_msgs::msg::Marker::ADD;
        select_waypoint.pose.position.x = x_traj[goal_index];
        select_waypoint.pose.position.y = y_traj[goal_index];
        select_waypoint.pose.position.z = 0;
        select_waypoint.pose.orientation.x = 0.0;
        select_waypoint.pose.orientation.y = 0.0;
        select_waypoint.pose.orientation.z = 0.0;
        select_waypoint.pose.orientation.w = 1.0;
        select_waypoint.scale.x = 0.4;
        select_waypoint.scale.y = 0.4;
        select_waypoint.scale.z = 0.4;
        select_waypoint.color.a = 1.0;
        select_waypoint.color.r = 1.0;
        select_waypoint.color.g = 0.0;
        select_waypoint.color.b = 0.0;
        
        markerpub->publish(select_waypoint);

        std::vector<double> goal_pose;
        goal_pose.push_back(x_traj[goal_index]);
        goal_pose.push_back(y_traj[goal_index]);
        goal_pose.push_back(yaw_traj[goal_index]);
        return goal_pose;
    }
    double clamp(double value, double min_value, double max_value) {
        return std::max(min_value, std::min(value, max_value));}

    double quaternionToYaw(const geometry_msgs::msg::Quaternion& quaternion) {
    tf2::Quaternion q(
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w
    );

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
    }

    void pose_callback(const nav_msgs::msg::Odometry::ConstPtr pose_msg)
    {
        // TODO: find the current waypoint to track using methods mentioned in lecture
        auto pose = pose_msg->pose.pose;
        std::vector<double> goal;
        goal = select_goal(pose);

        // TODO: transform goal point to vehicle frame of reference
        double dx = goal[0] - pose.position.x;
        double dy = goal[1] - pose.position.y;
        // double yaw = goal[2]; issues
        double yaw = quaternionToYaw(pose.orientation);

        double dx_trans = dx*cos(yaw) + dy*sin(yaw);
        double dy_trans = -dx*sin(yaw) + dy*cos(yaw);
 
        // TODO: calculate curvature/steering angle
        double  gamma = 2*dy_trans / pow(L,2);

        double steer = clamp(PGain*gamma, min_steer, max_steer);

        // TODO: publish drive message, don't forget to limit the steering angle.
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.speed = velocity;
        drive_msg.drive.steering_angle = steer;

        this->drivepub->publish(drive_msg);
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}