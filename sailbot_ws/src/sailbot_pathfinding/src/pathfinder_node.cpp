#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sailbot_msgs/srv/get_path.hpp"

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

#define NOGO_ANGLE_DEGREES 45

class Pathfinder : public rclcpp::Node
{
public:
    Pathfinder() : Node("pathfinder_node")
    {

    }

    std::vector<std::pair<double, double>> find_solution(Sailbot::Map map, double wind_angle_deg, Sailbot::Node* start_node, Sailbot::Node* goal_node) {
    double wind_angle_rad = wind_angle_deg * (M_PI / 180);
    double nogo_angle_rad = NOGO_ANGLE_DEGREES * (M_PI / 180);
    bool wind_blocked = false;
    //start with linear solver
    if (!is_in_nogo(start_node, goal_node, wind_angle_rad, nogo_angle_rad)) {
        LinearRaycastPathfindingStrategy linearSolver;
            auto path = linearSolver.solve(map, start_node, goal_node, wind_angle_rad, nogo_angle_rad);
            //return path;
            if (path.size() > 0) {
                    return path;
            }
    }
    else {
        wind_blocked = true;
    }
    //if that fails, try one tack
    if (wind_blocked) {
        OneTackPathfindingStrategy oneTackSolver;
        auto path = oneTackSolver.solve(map, start_node, goal_node, wind_angle_rad, nogo_angle_rad);
        if (path.size() > 0) {
            return path;
        }
    }
    //if both fail, fall back to pathfinding

    //create solver and solve
    AStarPathfindingStrategy solver;

    auto time_start = std::chrono::high_resolution_clock::now();
    auto path = solver.solve(map, start_node, goal_node, wind_angle_rad);
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