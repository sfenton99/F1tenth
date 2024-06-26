// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(): rclcpp::Node("rrt_node"), x_dist(0, gridHeight - 1), y_dist(-gridWidth / 2, gridWidth / 2 - 1), gen((std::random_device())()) {

    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need
    

    // ROS subscribers
    // TODO: create subscribers as you need
    string pose_topic = "ego_racecar/odom";
    string scan_topic = "/scan";
    string occupancy_topic = "/local_occupancy_map";
    string goal_topic = "/vis_goal";
    std::string drive_topic = "/drive";


    drivepub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
    goal_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(goal_topic, 10);
    grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(occupancy_topic, 10);
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pose_topic, 1, std::bind(&RRT::pose_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, 1, std::bind(&RRT::scan_callback, this, std::placeholders::_1));

    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Created new RRT Object.");

}
vector<float> preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg){

        std::vector<float> filteredData;
        int window_size = 0;
        float max_range = 100.0;
        //moving average filter
        if(int(scan_msg->ranges.size()) < window_size || window_size < 1) {
            // std::cout << "INVALID DATA OR WINDOW SIZE" << std::endl;
            return scan_msg->ranges;
        }
        double sum = 0.0;
        for(size_t i=0; i < scan_msg->ranges.size(); i++){
            sum +=scan_msg->ranges[i];
            if (int(i) >= window_size) {
                sum -= scan_msg->ranges[i - window_size]; //remove elements leaving the window
            }
            //Partial window for indices < window size
            filteredData.push_back(sum / std::min(int(i+1), window_size));
        }
        //smoothing max and sparse measurements
        for(size_t i=0; i < filteredData.size(); i++){
            if (std::isnan(filteredData[i]) || std::isinf(filteredData[i] || filteredData[i] > max_range)){
                filteredData[i] = max_range; //large number
            }
        }
        return filteredData;
    }

std::pair<int,int> find_max_gap(const std::vector<float> range_data)
    {   
        double threshold = 2;  //dist threshold to be called a gap
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

std::vector<float> find_best_point(std::vector<float> ranges, std::pair<int,int> gap, float angle_min, float angle_incr)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        int best_point_idx = floor((gap.first + gap.second)/2);
        vector<float> best_point;
        float angle = angle_min + best_point_idx*angle_incr;

        best_point.push_back(ranges[best_point_idx]); //r
        best_point.push_back(angle); //angle
        return best_point;
    }

double dist(RRT_Node &node, std::vector<double> &sampled_point) {
    double sum_of_squares = (node.x - sampled_point[0])*(node.x - sampled_point[0]) + (node.y - sampled_point[1])*(node.y - sampled_point[1]);
    return std::sqrt(sum_of_squares);
}

void RRT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //
    // Ureate occupancy grid
    // std::vector<float> ranges = scan_msg->ranges;

    // cout << "HERE" <<endl;
    RCLCPP_DEBUG(rclcpp::get_logger("RRT"), "%s\n", "ENTER SCAN_CALLBACK");
    std::vector<float> ranges_processed = preprocess_lidar(scan_msg);
    // Find max length gap 
    std::pair<int,int> gap = find_max_gap(ranges_processed); //two indices, start and end

    // Find the best point in the gap 
    goal = find_best_point(ranges_processed, gap, scan_msg->angle_min, scan_msg->angle_increment);


    for(int i=0; i<OccupancyGridSize;i++){Occupancy.push_back(0);};
    
    // Update occupancy grid
    for(size_t i=0; i< ranges_processed.size(); i++){
        float curr_angle = scan_msg->angle_min + i*scan_msg->angle_increment;

        if (curr_angle < 3.14/2 && curr_angle > -3.14/2){
        int x_point = int(cos(curr_angle)*ranges_processed[i]/resolution);
        int y_point = int(gridWidth/2 + sin(curr_angle)*ranges_processed[i]/resolution);

        if(x_point < gridWidth && x_point > 0 && y_point > 0 && y_point < gridHeight){
        Occupancy[x_point + gridWidth*y_point] = 100;

        // expand occupancyy around cell 
        for (int bx = -bufferCells; bx <= bufferCells; ++bx) {
            for (int by = -bufferCells; by <= bufferCells; ++by) {
                int nx = x_point + bx; // New x-coordinate considering the buffer
                int ny = y_point + by; // New y-coordinate considering the buffer

                // Check if the new points are within the grid bounds
                if (nx >= 0 && nx < gridWidth && ny >= 0 && ny < gridHeight) {
                    Occupancy[nx + ny * gridWidth] = 100; // Mark as occupied
        }
    }
}

        }
        }
    }
    nav_msgs::msg::OccupancyGrid grid_msg;
    grid_msg.data = Occupancy;
    string frame = "ego_racecar/base_link";
    grid_msg.header.frame_id = frame;
    grid_msg.info.width = gridWidth;
    grid_msg.info.height = gridHeight;
    grid_msg.info.resolution = resolution;  //can change later if not fast enough
    grid_msg.info.origin.position.y = -(gridWidth*resolution)/2;
    grid_msg.info.origin.position.z = 0.1;
    grid_msg.header.stamp = this->get_clock()->now();
    grid_pub_->publish(grid_msg);
    Occupancy.clear();
    
}

void RRT::visualize_goal(float &x, float &y){
    goal_point.header.frame_id = "ego_racecar/base_link";
    // goal_point.id = 0; //maybe remove
    goal_point.header.stamp = this->get_clock()->now();
    goal_point.ns = "goal";
    goal_point.type = visualization_msgs::msg::Marker::SPHERE;
    goal_point.action = visualization_msgs::msg::Marker::ADD;
    goal_point.pose.position.x = x;
    goal_point.pose.position.y = y;
    goal_point.pose.position.z = 0.1;
    goal_point.pose.orientation.x = 0.0;
    goal_point.pose.orientation.y = 0.0;
    goal_point.pose.orientation.z = 0.0;
    goal_point.pose.orientation.w = 1.0;
    goal_point.scale.x = 0.4;
    goal_point.scale.y = 0.4;
    goal_point.scale.z = 0.4;
    goal_point.color.a = 1.0;
    goal_point.color.r = 1.0;
    goal_point.color.g = 0.0;
    goal_point.color.b = 0.0;
    goal_pub_->publish(goal_point);

}

void RRT::visualize_path(vector<RRT_Node> &pathfound){
    // cout << "Here!" <<endl;
    path_marker.points.clear();
    path_marker.header.frame_id = "ego_racecar/base_link";
    // path_marker.id = 1; //maybe remove
    path_marker.header.stamp = this->get_clock()->now();
    path_marker.ns = "path_to_goal";
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;

    path_marker.scale.x = 0.05;

    path_marker.color.a = 1.0;
    path_marker.color.r = 0.0;
    path_marker.color.g = 0.0;
    path_marker.color.b = 1.0;

    geometry_msgs::msg::Point path_point = geometry_msgs::msg::Point();
    path_point.z = 0.1;
    for(size_t i=0; i<pathfound.size(); i++){
        path_point.x = float(pathfound[i].x)*resolution;
        path_point.y = float(pathfound[i].y)*resolution;
        path_marker.points.push_back(path_point);
    }   
        goal_pub_->publish(path_marker);
        
}
std::vector<double> RRT::select_goal(vector<RRT_Node> &pathfound, const geometry_msgs::msg::Pose &pose_curr){
        int goal_index;
        size_t closest_index;
        // double curr_x = pose_curr.position.x;
        // double curr_y = pose_curr.position.y;
        double min_dist = 1000;
        double len;

        // find closest point
        // for(size_t i=0; i<pathfound.size(); i+=step_size){
        //     len = sqrt(pow((pathfound[i].x-curr_x),2) + pow((pathfound[i].y-curr_y),2));
        //     if (len < min_dist) {
        //         closest_index = i;
        //         min_dist = len;
        //     }
        // }

        // ensure lookahead distance enforced
        // for(size_t i=closest_index; i<pathfound.size(); i++){
        //     len = sqrt(pow((pathfound[i].x-curr_x),2) + pow((pathfound[i].y-curr_y),2));
        //     if (len > L) {
        //         goal_index = int(i);
        //         break;
        //     }
        // }

        // ensure lookahead distance enforced remove curr pose
        for(size_t i=0; i<pathfound.size(); i++){
            // cout << "index: " << i << " x: " << pathfound[i].x*resolution << " y: " << pathfound[i].y*resolution << endl;
            len = sqrt(pow((pathfound[i].x* resolution),2) + pow((pathfound[i].y* resolution),2));
            // cout << "dist: " << len;
            if (len > L) {
                goal_index = int(i);
                break;
            }
        }
        // cout << "GOAL INDEX: " << goal_index << endl;

        std::vector<double> target = {pathfound[goal_index].x, pathfound[goal_index].y};
        target_marker.header.frame_id = "ego_racecar/base_link";;
        target_marker.header.stamp = this->get_clock()->now();
        target_marker.ns = "target";
        target_marker.type = visualization_msgs::msg::Marker::SPHERE;
        target_marker.action = visualization_msgs::msg::Marker::ADD;
        target_marker.pose.position.x = target[0]*resolution;
        target_marker.pose.position.y = target[1]*resolution;
        // target_marker.pose.position.x = gridHeight * cos(goal[1])*resolution;
        // target_marker.pose.position.y = gridHeight * sin(goal[1])*resolution;
        target_marker.pose.position.z = 0.1;
        target_marker.pose.orientation.x = 0.0;
        target_marker.pose.orientation.y = 0.0;
        target_marker.pose.orientation.z = 0.0;
        target_marker.pose.orientation.w = 1.0;
        target_marker.scale.x = 0.2;
        target_marker.scale.y = 0.2;
        target_marker.scale.z = 0.2;
        target_marker.color.a = 1.0;
        target_marker.color.r = 0.0;
        target_marker.color.g = 1.0;
        target_marker.color.b = 0.0;
        goal_pub_->publish(target_marker);
        visualize_path(pathfound);

        return target;

}
double clamp(double value, double min_value, double max_value) {
        return std::max(min_value, std::min(value, max_value));}

void RRT::pure_pursuit(vector<RRT_Node> &pathfound, const nav_msgs::msg::Odometry::ConstPtr &pose_msg){
    auto pose = pose_msg->pose.pose;
    std::vector<double> target;
    // for(size_t i=0; i<pathfound.size(); i++){
    //         cout << "index: " << i << " x: " << pathfound[i].x*resolution << " y: " << pathfound[i].y*resolution << endl;
    // }
    target = select_goal(pathfound, pose);

    // TODO: transform goal point to vehicle frame of reference
    // double dx = target[0];
    double dy = target[1];
    // double dx = gridHeight * cos(goal[1]);
    // double dy = gridHeight * sin(goal[1]); //test

    // TODO: calculate curvature/steering angle
    double  gamma = 2*dy / pow(L,2);

    double steer_angle = clamp(PGain*gamma, min_steer, max_steer);

    // TODO: publish drive message, don't forget to limit the steering angle.
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.drive.speed = velocity;
    drive_msg.drive.steering_angle = steer_angle;

    this->drivepub->publish(drive_msg);

}
void RRT::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //
    //restrict to max distance
    RCLCPP_DEBUG(rclcpp::get_logger("RRT"), "%s\n", "ENTER POSE_CALLBACK");
    goal_path.clear();
    float goal_x, goal_y;
    if (gridHeight <= abs(goal[0]/resolution)){
        goal_x =  gridHeight * cos(goal[1]); //grid world coordinates
        goal_y =  gridHeight * sin(goal[1]);}
    else{
        goal_x =  abs(goal[0]/resolution - bufferCells) * cos(goal[1]); //grid world coordinates
        goal_y =  abs(goal[0]/resolution - bufferCells) * sin(goal[1]);
    }

    float scaled_goal_x = goal_x*resolution;
    float scaled_goal_y = goal_y*resolution;
    visualize_goal(scaled_goal_x, scaled_goal_y); //real world coordinates

    // // tree as std::vector
    std::vector<RRT_Node> tree;

    RRT_Node start;
    start.x = 0;
    start.y = 0;
    start.parent = 0;
    start.is_root = true;
    

    // // TODO: fill in the RRT main loop
    for(int k=0; k<num_samples; k++){
        if(k==0){
            tree.push_back(start);
        }
        

        // sample new point
        std::vector<double> x_rand = sample();

        // find nearest member in tree
        int x_near_idx = nearest(tree, x_rand); // test next
       
        // create new node
        RRT_Node x_new = steer(tree[x_near_idx], x_rand);
        x_new.is_root = false;
        
        if (!check_collision(tree[x_near_idx], x_new)){

            x_new.parent = x_near_idx; //set parent
            
            // ------ASTAR REWIRE --------
            vector<int> neighbors = near(tree, x_new);
          
            double min_cost = cost(tree,tree[x_near_idx]) + line_cost(tree[x_near_idx], x_new);
            double curr_cost;
                 
            for (size_t i=0; i < neighbors.size(); i++){
                if (!check_collision(tree[neighbors[i]], x_new)){
                    curr_cost = cost(tree, tree[neighbors[i]]) + line_cost(tree[neighbors[i]], x_new);
                    if (curr_cost < min_cost){
                        x_new.parent = neighbors[i];
                        min_cost = curr_cost;
                    }
                }
            } 


            if(is_goal(x_new, goal_x, goal_y)){     //if is goal find path to goal
                goal_path = find_path(tree, x_new);
                pure_pursuit(goal_path, pose_msg);
                tree.clear();
                break;
            }

            tree.push_back(x_new);

            for(size_t i=0; i < neighbors.size(); i++){
                min_cost = cost(tree, x_new) + line_cost(x_new, tree[neighbors[i]]); //new cost to neighbor

                if (min_cost < cost (tree, tree[neighbors[i]])){
                    if(!check_collision(x_new, tree[neighbors[i]])){
                        tree[neighbors[i]].parent = tree.size()-1;
                    }
                }

            }
           
        }
    }  
}

std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space
    
    std::vector<double> sampled_point;
    double random = double(x_dist(gen));
    if (random == 0.0) random += 1.0;
    sampled_point.push_back(random);
    random = double(y_dist(gen));
    if (random == 0.0) random += 1.0;
    sampled_point.push_back(random);
    
    return sampled_point;
}


int RRT::nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<RRT_Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node;
    double min_dist = 1000;
    double distance;
    // TODO: fill in this method
    for(size_t i=0; i<tree.size();i++){
        distance = dist(tree[i], sampled_point);
        if (distance < min_dist){
            min_dist = distance;
            nearest_node = i;
        }
    }
    return nearest_node;
}

RRT_Node RRT::steer(RRT_Node &nearest_node, std::vector<double> &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (RRT_Node): new node created from steering

    RRT_Node new_node;
    
    
    if (dist(nearest_node, sampled_point) < epsilon)   
    {   
        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
    
        if(new_node.x == 0 && new_node.y == 0){
            cout << "issue is in steer else statement" << endl;
        }
    }
    else{
        std::vector<double> near_point = {nearest_node.x, nearest_node.y};

        vector<double> diff_vector = {sampled_point[0] - near_point[0], sampled_point[1] - near_point[1]};
        float norm_of_diff_vector = dist(nearest_node, sampled_point);
        vector<double> unit_diff_vector = {diff_vector[0]/norm_of_diff_vector, diff_vector[1]/norm_of_diff_vector};
        new_node.x = unit_diff_vector[0]*epsilon + near_point[0];
        new_node.y = unit_diff_vector[1]*epsilon + near_point[1];

        if(new_node.x == 0 && new_node.y == 0){
            cout << "issue is in steer else statement" << endl;
        }
    }

    return new_node;
}
bool RRT::is_occupied(int &x, int &y){
    int new_y = y + gridHeight/2;
    if(x < gridWidth && x >= 0 && new_y >= 0 && new_y < gridHeight){
    if (Occupancy[x + gridWidth*new_y] > 50){
        // cout << "(" << x*resolution << "," << y*resolution << ") occupied" << endl;
        return true;
    }
    else{
        // cout << "(" << x*resolution << "," << y*resolution << ") free" << endl;
        return false;
    }
    }
    else{
        cout << "INVALID INDEX" << endl;
        cout << "x = " << x << endl;
        cout << "y = " << y << endl;
        cout << "new y = " << new_y << endl;
    }
}

bool RRT::check_collision(RRT_Node &nearest_node, RRT_Node &new_node) {
    // This method returns a boolean indicating if the path between the 
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    new_node (RRT_Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    bool collision = false;
    
    // Bresenhams line algo for collision checking
    int x_start = int(nearest_node.x);
    int y_start = int(nearest_node.y);
    int x_end = int(new_node.x);
    int y_end = int(new_node.y);

    int dx = std::abs(x_end - x_start);
    int dy = -std::abs(y_end - y_start);
    int sx, sy;
    if (x_start < x_end){sx = 1;}
    else{sx = -1;}
    if (y_start < y_end){sy = 1;}
    else{sy = -1;}
    
    int err = dx + dy; // error value e_xy
    int e2;

    while (true) {
        if (is_occupied(x_start, y_start)) { // Assume this function exists and returns true if the cell is occupied
            collision = true;
            break;
        }
        if (x_start == x_end && y_start == y_end){
            break;
        }
        e2 = 2*err;
        if (e2 >= dy) {
            err += dy;
            x_start += sx; }
        if (e2 <= dx) {
            err += dx;
            y_start += sy; }
    }

    return collision;
}

bool RRT::is_goal(RRT_Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    // TODO: fill in this method
    vector<double> goal = {goal_x, goal_y};
    if (dist(latest_added_node, goal) <= epsilon){
        close_enough = true;
    }

    return close_enough;
}

std::vector<RRT_Node> RRT::find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<RRT_Node>): the vector that represents the order of
    //      of the nodes traversed as the found path
    
    std::vector<RRT_Node> found_path;
    RRT_Node current_node = latest_added_node;
    found_path.push_back(current_node);

    // TODO: fill in this method
    while(!current_node.is_root){
        current_node = tree[current_node.parent];
        found_path.push_back(current_node);
    }
  
    std::reverse(found_path.begin(), found_path.end());
    return found_path;
}

// RRT* methods
double RRT::cost(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<RRT_Node>): the current tree
    //    node (RRT_Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    RRT_Node node_copy = node;
    while(!node_copy.is_root){
        RRT_Node parent = tree[node_copy.parent];
        cost += line_cost(node_copy, parent);
        node_copy = tree[node_copy.parent]; 
    }
    // cout << "cost: " << cost << endl;
    return cost;
}

double RRT::line_cost(RRT_Node &n1, RRT_Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (RRT_Node): the RRT_Node at one end of the path
    //    n2 (RRT_Node): the RRT_Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    // TODO: fill in this method
    double cost = std::sqrt((n1.x - n2.x)*(n1.x - n2.x) + (n1.y - n2.y)*(n1.y - n2.y));

    return cost;
}

std::vector<int> RRT::near(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<RRT_Node>): the current tree
    //   node (RRT_Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    // TODO:: fill in this method
    for(size_t i=0; i < tree.size(); i++){
        double distance = line_cost(tree[i], node);
        if (distance < radius){
            neighborhood.push_back(i);
   
        }
    }
           
    return neighborhood;
}