#include "Planning.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/srv/get_map.hpp"
    

PlanningNode::PlanningNode() :
    rclcpp::Node("planning_node") {

    // Creating the client for fetching the map
    map_client_ = this->create_client<nav_msgs::srv::GetMap>("map_server/map");

    RCLCPP_INFO(get_logger(), "Planning node started.");

    // Creating the service for path planning
    plan_service_ = this->create_service<nav_msgs::srv::GetPlan>("plan_path",
        std::bind(&PlanningNode::planPath, this, std::placeholders::_1, std::placeholders::_2));

    // Creating the publisher for the planned path
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);

    // Connect to the map server
    while (!map_client_->wait_for_service(std::chrono::seconds(2))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for map service. Exiting.");
            return;
        }
        RCLCPP_INFO(get_logger(), "Map service not available, waiting...");
    }

    // Requesting the map
    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
    auto future = map_client_->async_send_request(request, std::bind(&PlanningNode::mapCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Trying to fetch map...");
}

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
    auto response = future.get();

    if (response) {
        RCLCPP_INFO(get_logger(), "Map successfully received!");
        map_ = response->map;

        // Example: Print map dimensions
        RCLCPP_INFO(get_logger(), "Map size: width = %d, height = %d", map_.info.width, map_.info.height);

        // Dilate the map for obstacle expansion
        dilateMap();
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to receive map.");
    }
}

void PlanningNode::planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request, std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {
    
    RCLCPP_INFO(get_logger(), "Received path planning request from (%f, %f) to (%f, %f)",
    request->start.pose.position.x, request->start.pose.position.y,
    request->goal.pose.position.x, request->goal.pose.position.y);

    // Run A-Star algorithm to find the path
    aStar(request->start, request->goal);

    // Smooth the generated path
    smoothPath();

    // Publish the computed path
    path_pub_->publish(path_);

    // Send the planned path as a response
    response->plan = path_;

    RCLCPP_INFO(get_logger(), "Path planning completed and sent back.");
}


void PlanningNode::dilateMap() {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    nav_msgs::msg::OccupancyGrid dilatedMap = map_;
    ... processing ...
    map_ = dilatedMap;
    */
}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) {
    RCLCPP_INFO(get_logger(), "Starting A-Star path planning...");

    // Convert start and goal positions to map indices
    int start_x = (start.pose.position.x - map_.info.origin.position.x) / map_.info.resolution;
    int start_y = (start.pose.position.y - map_.info.origin.position.y) / map_.info.resolution;
    int goal_x = (goal.pose.position.x - map_.info.origin.position.x) / map_.info.resolution;
    int goal_y = (goal.pose.position.y - map_.info.origin.position.y) / map_.info.resolution;

    Cell cStart(start_x, start_y);
    Cell cGoal(goal_x, goal_y);

    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(map_.info.height * map_.info.width, false);

    openList.push_back(std::make_shared<Cell>(cStart));

    // Define movement directions: up, down, left, right, diagonals
    std::vector<std::pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1},
                                                   {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    while (!openList.empty() && rclcpp::ok()) {
        // Find the cell with the lowest f-value in openList
        auto current = *std::min_element(openList.begin(), openList.end(),
                                         [](const std::shared_ptr<Cell>& a, const std::shared_ptr<Cell>& b) {
                                             return a->f < b->f;
                                         });

        // Remove current from openList and mark it as visited
        openList.erase(std::remove_if(openList.begin(), openList.end(),
                                      [&current](const std::shared_ptr<Cell>& c) { return c->x == current->x && c->y == current->y; }),
                       openList.end());

        closedList[current->y * map_.info.width + current->x] = true;

        // If the goal is reached, reconstruct the path
        if (current->x == cGoal.x && current->y == cGoal.y) {
            RCLCPP_INFO(get_logger(), "Path found! Reconstructing...");
            
            nav_msgs::msg::Path planned_path;
            planned_path.header.stamp = this->get_clock()->now();
            planned_path.header.frame_id = "map";  // Ensure correct frame ID

            while (current) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = current->x * map_.info.resolution + map_.info.origin.position.x;
                pose.pose.position.y = current->y * map_.info.resolution + map_.info.origin.position.y;
                planned_path.poses.push_back(pose);
                current = current->parent;
            }

            std::reverse(planned_path.poses.begin(), planned_path.poses.end());

            // Explicitly publish to "planned_path" topic
            path_pub_->publish(planned_path);
            
            RCLCPP_INFO(get_logger(), "Path published to 'planned_path'. Planning complete.");
            return;
        }

        // Expand neighbors
        for (const auto& dir : directions) {
            int nx = current->x + dir.first;
            int ny = current->y + dir.second;

            // Ensure within bounds
            if (nx < 0 || ny < 0 || nx >= map_.info.width || ny >= map_.info.height) {
                continue;
            }

            // Check occupancy grid (0 = free space, otherwise obstacle)
            if (map_.data[ny * map_.info.width + nx] != 0) {
                continue;
            }

            // Skip if already visited
            if (closedList[ny * map_.info.width + nx]) {
                continue;
            }

            auto neighbor = std::make_shared<Cell>(nx, ny);
            neighbor->g = current->g + std::hypot(dir.first, dir.second);
            neighbor->h = std::hypot(nx - cGoal.x, ny - cGoal.y);
            neighbor->f = neighbor->g + neighbor->h;
            neighbor->parent = current;

            // Ensure no duplicate entries in openList
            bool skip = false;
            for (const auto& cell : openList) {
                if (cell->x == neighbor->x && cell->y == neighbor->y && cell->g <= neighbor->g) {
                    skip = true;
                    break;
                }
            }

            if (!skip) {
                openList.push_back(neighbor);
            }
        }
    }

    RCLCPP_ERROR(get_logger(), "Unable to plan path.");
}


void PlanningNode::smoothPath() {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;
    ... processing ...
    path_.poses = newPath;
    */
}

Cell::Cell(int c, int r) {
    // add code here
}
