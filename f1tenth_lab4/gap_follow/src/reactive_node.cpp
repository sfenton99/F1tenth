#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"


using std::placeholders::_1;

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        // Topics
        std::string lidarscan_topic = "/scan";
        std::string drive_topic = "/drive";
        std::string odometry_topic = "/ego_racecar/odom";

        // TODO: create ROS subscribers and publishers
        drivepub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
        lasersub = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 10, std::bind(&ReactiveFollowGap::scan_callback, this, _1));
        // odomsub = this->create_subscription<nav_masgs::msg::Odemetry>(odometry_topic, 10, std::bind(&WallFollow::odometry_callback, this, _1));
    }

private:
    //TUNABLE PARAMETERS
    int window_size = 2;  //moving average filter on lidar data
    int threshold = 2.5;  //dist threshold to be called a gap
    int min_wall_dist = 1.0; // if dist to wall less than val, wont turn
    float Kp = 0.5;
    float Kd = 0;
    double L = 0.2; //lookahead distance
    double dist_to_wall = 0.6; //test

    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lasersub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drivepub;

    std::vector<float> MovingAverageFiler(const std::vector<float>& data, int windowSize){
        std::vector<float> filteredData;

        if(int(data.size()) < windowSize || windowSize < 1) {
            std::cout << "INVALID WINDOW SIZE" << std::endl;
            return filteredData;
        }

        double sum = 0.0;
        for(size_t i=0; i < data.size(); i++){
            sum +=data[i];
            if (int(i) >= windowSize) {
                sum -= data[i - windowSize]; //remove elements leaving the window
            }
            //Partial window for indices < window size
            filteredData.push_back(sum / std::min(int(i+1), windowSize));
        }
        return filteredData;
    }
    
    std::vector<float> preprocess_lidar(std::vector<float> range_data)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
        int window_size = this->window_size;

        //Handle Nans and Inf
        for(size_t i=0; i < range_data.size(); i++){
            if (std::isnan(range_data[i]) || std::isinf(range_data[i])){
                range_data[i] = 10; //large number
            }
        }
        // moving average filter
        std::vector<float> filteredLidarData = MovingAverageFiler(range_data, window_size);
        return filteredLidarData;
    }

    std::pair<int,int> find_max_gap(const std::vector<float> range_data)
    {   
        int threshold = this->threshold;
        int largest_gap = 0;
        int gap = 0;
        std::pair<int,int> indices; //start and end indices as fixed size array to return
        for(size_t i=0; i<range_data.size(); i++){
            if (range_data[i]>threshold){
                gap +=1;
                if (gap > largest_gap){
                    indices.second = i;
                    largest_gap = gap;
                }
            }
            else if(range_data[i]<= threshold){
                gap = 0;
            }
        }
        indices.first = indices.second - largest_gap;
        return indices;
    }

    int find_best_point(std::vector<float> ranges, std::pair<int,int> gap)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        int best_point = gap.first;
        for(int i=gap.first; i<=gap.second; i++){
            if(ranges[i]>ranges[best_point]){
                best_point=i;
            }
        }
        return best_point;
        // return floor((gap.first + gap.second)/2);
    }
    double get_range(const std::vector<float> range_data, double angle, double min_angle, double angle_increment)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.
        */
        int index = static_cast<int>((angle-min_angle)/(angle_increment));
        return range_data[index];
    }

    double get_error(double a_range, double b_range)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()
        */
        double theta = abs(30 * (3.14 / 180) - 90 * (3.14 / 180));
        double alpha = atan((a_range*cos(theta) - b_range)/a_range*sin(theta));
        double Dt = b_range * cos(alpha);
        double Dtplus1 = Dt + this->L*sin(alpha);
        // std::cout << "A range: " << a_range << std::endl;
        // std::cout << "B range: " << b_range << std::endl;
        
        return(Dtplus1 - this->dist_to_wall);
    }

    float controller(float heading_error_gap, double heading_error_wall, std::vector<float> ranges, float min_angle, float angle_increment){
        float u=0;
        int left_index = floor(((90*3.14/180) - min_angle)/angle_increment);
        int right_index = floor(((-90*3.14/180) - min_angle)/angle_increment);
         std::cout << "Left Side: " << ranges[left_index] << " Right Side: "<< ranges[right_index]<< std::endl;


        if (ranges[left_index] < this->min_wall_dist || ranges[right_index] < this->min_wall_dist){
            u = this->Kp*heading_error_wall;
            std::cout << "WALL FOLLOWING" << std::endl;
        }
        else{
        u = this->Kp*heading_error_gap;
        std::cout << "GAP FOLLOWING" << std::endl;
        }

        return u;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        /// TODO:
        // Find closest point to LiDAR
        // Eliminate all points inside 'bubble' (set them to zero) 
        std::vector<float> ranges_processed = preprocess_lidar(scan_msg->ranges);

        // Find max length gap 
        std::pair<int,int> gap = find_max_gap(ranges_processed); //two indices, start and end

        // Find the best point in the gap 
        int best_point = find_best_point(ranges_processed, gap);

        // Get heading angle associated with gap point
        float heading_err_gap = scan_msg->angle_min + best_point*scan_msg->angle_increment;
        
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();

        //mix in some wall following
        double a_scan_angle = 30 * (3.14 / 180);
        double b_scan_angle = 90 * (3.14 / 180);
        double a_range = get_range(scan_msg->ranges, a_scan_angle, scan_msg->angle_min, scan_msg->angle_increment);
        double b_range = get_range(scan_msg->ranges, b_scan_angle, scan_msg->angle_min, scan_msg->angle_increment);
        double heading_err_wall = get_error(a_range, b_range);

        //arbitrate and control
        float u = controller(heading_err_gap, heading_err_wall, scan_msg->ranges, scan_msg->angle_min, scan_msg->angle_increment);

        int steering_angle_degrees = int(heading_err_gap*180/3.14);

        std::cout << "Turning: " << steering_angle_degrees << " Degrees" << std::endl;
        // std::cout << "Deepest point in void: " << scan_msg->ranges[best_point] << " m" << std::endl;
        // Publish Drive message
        float velocity;
         if ((abs(steering_angle_degrees) >= 0) && (abs(steering_angle_degrees) <= 10)){
            velocity = 1.5;
        } else if ((abs(steering_angle_degrees) > 10) && (abs(steering_angle_degrees) <= 20)){
            velocity = 1.0;
        } else if (abs(steering_angle_degrees) > 20){
            velocity = 0.5;
        }

        drive_msg.drive.speed = velocity;
        drive_msg.drive.steering_angle = u;

        this->drivepub->publish(drive_msg);
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}