#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>

using std::placeholders::_1;

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // Topics
        std::string lidarscan_topic = "/scan";
        std::string drive_topic = "/drive";

        // TODO: create ROS subscribers and publishers
        drivepub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
        lasersub = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 10, std::bind(&WallFollow::scan_callback, this, _1));
        // odomsub = this->create_subscription<nav_masgs::msg::Odemetry>("/ego_racecar/odom", 10, std::bind(&WallFollow::odometry_callback, this, _1));
    }

private:
    // PID CONTROL PARAMS
    double kp = 1.2;
    double kd = 0.8;
    double ki = 0.000001;
    double dt= 0.0;
    double dist_to_wall = 1.0;
    double L = 0.2; //lookahead distance
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lasersub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drivepub;

    double curr_scan_timestamp = 0.0;


    double get_range(const std::vector<float> range_data, double angle, double min_angle, double angle_increment)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */
        int index = static_cast<int>((angle-min_angle)/(angle_increment));
        return range_data[index];
    }

    double get_error(double a_range, double b_range)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        double theta = abs(30 * (3.14 / 180) - 90 * (3.14 / 180));
        double alpha = atan((a_range*cos(theta) - b_range)/a_range*sin(theta));
        double Dt = b_range * cos(alpha);
        double Dtplus1 = Dt + this->L*sin(alpha);
        // std::cout << "A range: " << a_range << std::endl;
        // std::cout << "B range: " << b_range << std::endl;
        
        return(Dtplus1 - this->dist_to_wall);
    }

    void pid_control(double error)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */
        double u = 0.0;
        double velocity;

        u = this->kp*error + this->ki*this->integral;
        
        // if multiple message, add derivative term
        if (this->dt != 0.0){
        u += this->kd*((error - this->prev_error)/(this->dt*pow(10.0,-9.0)));}

        std::cout << "error: " << error << "   |   prev error: " << this->prev_error << std::endl;
        std::cout << "dt: " << dt << "nanosecs" << std::endl;
        std::cout << "e_dot: " << ((error - this->prev_error)/(this->dt*pow(10.0,-9.0))) << std::endl;

        // TODO: Use kp, ki & kd to implement a PID controller
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // TODO: fill in drive message and publish

         if ((abs(u) >= 0) && (abs(u) < 10)){
            velocity = 1.5;
        } else if ((abs(u) > 10) && (abs(u) < 20)){
            velocity = 1.0;
        } else if (abs(u) > 20){
            velocity = 0.5;
        }

        drive_msg.drive.speed = velocity;
        drive_msg.drive.steering_angle = u;

        this->drivepub->publish(drive_msg);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        double a_scan_angle = 30 * (3.14 / 180);
        double b_scan_angle = 90 * (3.14 / 180);

        double a_range = get_range(scan_msg->ranges, a_scan_angle, scan_msg->angle_min, scan_msg->angle_increment);
        double b_range = get_range(scan_msg->ranges, b_scan_angle, scan_msg->angle_min, scan_msg->angle_increment);

        if (std::isfinite(a_range) && std::isfinite(b_range)) //not Inf or Nan
        {
        this->prev_error = this->error;
        this->error = get_error(a_range, b_range); // degrees
        this->integral += this->error;
        
        
        if(this->curr_scan_timestamp != 0.0){
            this->dt = scan_msg->header.stamp.nanosec - this->curr_scan_timestamp ;
        }
        this->curr_scan_timestamp = scan_msg->header.stamp.nanosec;
        std::cout << "distance to wall: " << b_range << std::endl;
    
        pid_control(this->error);
        }
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}