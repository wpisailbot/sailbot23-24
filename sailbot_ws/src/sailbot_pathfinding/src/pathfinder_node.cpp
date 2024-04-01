#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sailbot_msgs/srv/get_path.hpp"
#include "sailbot_msgs/srv/set_map.hpp"

#include <chrono>
#include <random>
#define _USE_MATH_DEFINES
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include "map.h"
#include "linear_raycast_pathfinding_strategy.h"
#include "one_tack_pathfinding_strategy.h"
#include "astar_pathfinding_strategy.h"
#include "utilities.h"

class Pathfinder : public rclcpp::Node
{
public:
    std::unique_ptr<Sailbot::Map> pMap = nullptr;
    rclcpp::Service<sailbot_msgs::srv::SetMap>::SharedPtr pSetMapService;
    rclcpp::Service<sailbot_msgs::srv::GetPath>::SharedPtr pGetPathService;

    Pathfinder() : Node("pathfinder_node")
    {
        this->pSetMapService = this->create_service<sailbot_msgs::srv::SetMap>(
            "set_map",
            std::bind(&Pathfinder::handle_set_map_service, this, std::placeholders::_1, std::placeholders::_2)
        );
        this->pGetPathService = this->create_service<sailbot_msgs::srv::GetPath>(
            "get_path", 
            std::bind(&Pathfinder::handle_get_path_service, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

    void handle_get_path_service(
        const std::shared_ptr<sailbot_msgs::srv::GetPath::Request> request,
        std::shared_ptr<sailbot_msgs::srv::GetPath::Response> response)
    {
        if(pMap == nullptr){
            RCLCPP_WARN(this->get_logger(), "GetPath called, but no map has been set!");

        }
        RCLCPP_INFO(this->get_logger(), "Calculating solution");
        uint x1 = request->start.x+pMap->half_width_diff;
        uint y1 = request->start.y+pMap->half_height_diff;
        uint x2 = request->end.x+pMap->half_width_diff;
        uint y2 = request->end.y+pMap->half_height_diff;
        RCLCPP_INFO(this->get_logger(), "Adjusted cells: (%d, %d), (%d, %d)", x1, y1, x2, y2);
        auto path = find_solution(*pMap, request->wind_angle_deg, pMap->getNode(x1, y1), pMap->getNode(x2, y2));
        RCLCPP_INFO(this->get_logger(), "Solution complete");
        for(auto p: path){
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = p.first;
            pose.pose.position.y = p.second; 
            response->path.poses.push_back(pose);
        }
        RCLCPP_INFO(this->get_logger(), "Sending back response");
    }

    void handle_set_map_service(
        const std::shared_ptr<sailbot_msgs::srv::SetMap::Request> request,
        [[maybe_unused]] std::shared_ptr<sailbot_msgs::srv::SetMap::Response> response){
        pMap = std::make_unique<Sailbot::Map>(uint32_t(request->map.info.width), uint32_t(request->map.info.height), request->map.data);
        for(uint32_t i=0; i<request->map.info.width*request->map.info.height; i++){
            if(request->map.data[i]==0){
                RCLCPP_INFO(this->get_logger(), "Found open cell");
            }
        }
        //pMap->data = pMap->data;
        RCLCPP_INFO(this->get_logger(), "Map set successfully");
    }

    std::vector<std::pair<double, double>> find_solution(Sailbot::Map& map, double wind_angle_deg, Sailbot::Node* start_node, Sailbot::Node* goal_node) {
    double wind_angle_rad = wind_angle_deg * (M_PI / 180);
    double nogo_angle_rad = NOGO_ANGLE_DEGREES * (M_PI / 180);
    bool wind_blocked = false;
    //start with linear solver
    if (!is_in_nogo(start_node, goal_node, wind_angle_rad, nogo_angle_rad)) {
        LinearRaycastPathfindingStrategy linearSolver;
        RCLCPP_INFO(this->get_logger(), "Trying linear raycast");

            auto path = linearSolver.solve(map, start_node, goal_node, wind_angle_rad, nogo_angle_rad);
            //return path;
            if (path.size() > 0) {
                    return path;
            }
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Path is upwind");
        wind_blocked = true;
    }
    //if that fails, try one tack
    if (wind_blocked) {
        RCLCPP_INFO(this->get_logger(), "Trying one tack");
        OneTackPathfindingStrategy oneTackSolver;
        auto path = oneTackSolver.solve(map, start_node, goal_node, wind_angle_rad, nogo_angle_rad);
        if (path.size() > 0) {
            return path;
        }
    }
    //if both fail, fall back to pathfinding
    RCLCPP_INFO(this->get_logger(), "Falling back to A-star");
    //create solver and solve
    AStarPathfindingStrategy solver;

    auto time_start = std::chrono::high_resolution_clock::now();
    auto path = solver.solve(map, start_node, goal_node, wind_angle_rad);
    RCLCPP_INFO(this->get_logger(), "Got A-star path");
    path = simplify_path(path, wind_angle_deg, map);
    auto time_stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(time_stop - time_start);
    std::string time_string = "Search time: " + std::to_string(duration.count());
    RCLCPP_INFO(this->get_logger(), time_string.c_str());

    if (path.size() == 0) {
        RCLCPP_INFO(this->get_logger(), "Path length is zero!");
        std::string start_string = "start: " + std::to_string(start_node->x) + ", " + std::to_string(start_node->y);
        std::string goal_string = "goal: " + std::to_string(goal_node->x) + ", " + std::to_string(goal_node->y);
        RCLCPP_INFO(this->get_logger(), start_string.c_str());
        RCLCPP_INFO(this->get_logger(), goal_string.c_str());
        return path;
    }
    return {};
}
private:

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Pathfinder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}