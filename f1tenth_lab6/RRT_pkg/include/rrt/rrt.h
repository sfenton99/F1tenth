// RRT assignment

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
#include <cmath>
#include <vector>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <tf2_ros/transform_broadcaster.h>

/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

// Struct defining the RRT_Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
typedef struct RRT_Node {
    double x, y;
    double cost; // only used for RRT*
    int parent; // index of parent node in the tree vector
    bool is_root = false;
} RRT_Node;


class RRT : public rclcpp::Node {
public:
    RRT();
    virtual ~RRT();
private:
    //PurePursuit Stuff
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drivepub;
    //TUNABLE PARAMETERS
    double L = 2; //lookahead distance
    double PGain = 0.1;
    double velocity = 0.3;
    int step_size = 1; //step size for waypoint selection
    double max_steer = 3.14/2;
    double min_steer = -3.14/2;
    void pure_pursuit(vector<RRT_Node> &path_found, const nav_msgs::msg::Odometry::ConstPtr &pose_msg);
    std::vector<double> select_goal(vector<RRT_Node> &pathfound, const geometry_msgs::msg::Pose &pose_curr);




    // TODO: add the publishers and subscribers you need
    visualization_msgs::msg::Marker goal_point = visualization_msgs::msg::Marker();
    visualization_msgs::msg::Marker target_marker = visualization_msgs::msg::Marker();
    visualization_msgs::msg::Marker path_marker = visualization_msgs::msg::Marker();
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    

    // TODO: create a occupancy grid
    const int gridWidth = 100;
    const int gridHeight = 100;
    const double resolution = 0.05; //same as map
    const int OccupancyGridSize = (gridWidth*gridHeight);

    //goal
    vector<float> goal;
    vector<RRT_Node>goal_path;
    int radius = 10; //radius for rrt star

    const int num_samples = 500; //rrt
    const double epsilon = 5; //rrt in grid world
    std::vector<signed char> Occupancy;
    int bufferCells = 3;

    // random generator, use this
    std::mt19937 gen;
    std::uniform_int_distribution<int> x_dist; //only forward, validate
    std::uniform_int_distribution<int> y_dist;
    double range = 2.0; //range around car pos for sample
    
    //publisher function
    void visualize_goal(float &x, float &y);

    void visualize_path(vector<RRT_Node> &path_found);
    // callbacks
    // where rrt actually happens
    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);

    // updates occupancy grid
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    // RRT methods
    bool is_occupied(int &x, int &y);
    std::vector<double> sample();
    int nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point);
    RRT_Node steer(RRT_Node &nearest_node, std::vector<double> &sampled_point);
    bool check_collision(RRT_Node &nearest_node, RRT_Node &new_node);
    bool is_goal(RRT_Node &latest_added_node, double goal_x, double goal_y);
    std::vector<RRT_Node> find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node);
    // RRT* methods
    double cost(std::vector<RRT_Node> &tree, RRT_Node &node);
    double line_cost(RRT_Node &n1, RRT_Node &n2);
    std::vector<int> near(std::vector<RRT_Node> &tree, RRT_Node &node);

};

