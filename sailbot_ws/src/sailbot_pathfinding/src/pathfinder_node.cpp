#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sailbot_msgs/srv/get_path.hpp"
#include "sailbot_msgs/srv/set_map.hpp"
#include "sailbot_msgs/srv/set_threat.hpp"

#include <chrono>
#include <random>
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include "map.h"
#include "pathfinding_strategy_base.h"
#include "linear_raycast_pathfinding_strategy.h"
#include "one_tack_pathfinding_strategy.h"
#include "astar_pathfinding_strategy.h"
#include "prm_pathfinding_strategy.h"
#include "utilities.h"

#define NOGO_ANGLE_DEGREES 45

class Pathfinder : public rclcpp::Node
{
public:
    std::vector<std::pair<Threat, cv::Mat>> threats;
    cv::Mat threatsMap;
    std::unique_ptr<Map> pMap = nullptr;
    rclcpp::Service<sailbot_msgs::srv::SetMap>::SharedPtr pSetMapService;
    rclcpp::Service<sailbot_msgs::srv::GetPath>::SharedPtr pGetPathService;
    rclcpp::Service<sailbot_msgs::srv::SetThreat>::SharedPtr pSetThreatService;

    Pathfinder() : Node("pathfinder_node")
    {
        this->pSetMapService = this->create_service<sailbot_msgs::srv::SetMap>(
            "set_map",
            std::bind(&Pathfinder::handle_set_map_service, this, std::placeholders::_1, std::placeholders::_2));
        this->pGetPathService = this->create_service<sailbot_msgs::srv::GetPath>(
            "get_path",
            std::bind(&Pathfinder::handle_get_path_service, this, std::placeholders::_1, std::placeholders::_2));
        this->pSetThreatService = this->create_service<sailbot_msgs::srv::SetThreat>(
            "set_threat",
            std::bind(&Pathfinder::handle_set_threat_service, this, std::placeholders::_1, std::placeholders::_2));
    }

    void handle_get_path_service(
        const std::shared_ptr<sailbot_msgs::srv::GetPath::Request> request,
        std::shared_ptr<sailbot_msgs::srv::GetPath::Response> response)
    {
        if (pMap == nullptr)
        {
            RCLCPP_WARN(this->get_logger(), "GetPath called, but no map has been set!");
        }
        RCLCPP_INFO(this->get_logger(), "Calculating solution");
        float x1f = request->start.x + pMap->half_width_diff;
        float y1f = request->start.y + pMap->half_height_diff;
        float x2f = request->end.x + pMap->half_width_diff;
        float y2f = request->end.y + pMap->half_height_diff;
        if(x1f<0.0 || x2f<0.0 || y1f<0.0 || y2f>0.0){
            RCLCPP_WARN(this->get_logger(), "Cells out of map bounds!");
            return;
        }

        uint x1 = uint(x1f);
        uint y1 = uint(y1f);
        uint x2 = uint(x2f);
        uint y2 = uint(y2f);
        RCLCPP_INFO(this->get_logger(), "Adjusted cells: (%d, %d), (%d, %d)", x1, y1, x2, y2);
        RCLCPP_INFO(this->get_logger(), "Map dims: (%d, %d)", pMap->height, pMap->height);

        if(x1>=pMap->width || x2>=pMap->width || y1>=pMap->height || y2>=pMap->height){
            RCLCPP_WARN(this->get_logger(), "Cells out of map bounds!");
            return;
        }
        auto path = find_solution(*pMap, request->wind_angle_deg, pMap->getMapNode(x1, y1), pMap->getMapNode(x2, y2), request->pathfinding_strategy);
        RCLCPP_INFO(this->get_logger(), "Solution complete");
        for (auto p : path)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = p.first - pMap->half_width_diff;
            pose.pose.position.y = p.second - pMap->half_height_diff;
            response->path.poses.push_back(pose);
        }
        RCLCPP_INFO(this->get_logger(), "Sending back response");
    }

    void handle_set_map_service(
        const std::shared_ptr<sailbot_msgs::srv::SetMap::Request> request,
        [[maybe_unused]] std::shared_ptr<sailbot_msgs::srv::SetMap::Response> response)
    {
        pMap = std::make_unique<Map>(uint32_t(request->map.info.width), uint32_t(request->map.info.height), request->map.data);
        pMap->initPRM(request->num_prm_nodes, request->prm_connection_distance_percent);
        threatsMap = cv::Mat::zeros(pMap->max_dim, pMap->max_dim, CV_32FC1);
        // pMap->data = pMap->data;
        RCLCPP_INFO(this->get_logger(), "Map set successfully");
    }

    void handle_set_threat_service(
        const std::shared_ptr<sailbot_msgs::srv::SetThreat::Request> request,
        [[maybe_unused]] std::shared_ptr<sailbot_msgs::srv::SetThreat::Response> response)
    {
        if (pMap == nullptr)
        {
            RCLCPP_WARN(this->get_logger(), "SetThreat failed: Cannot add threats before a map is set");
        }
        if (request->remove)
        {
            try
            {
                Threat threat;
                cv::Mat mat;
                threats.at(request->id) = std::make_pair(threat, mat);
                regenerateThreatMap();
                return;
            }
            catch (const std::out_of_range &e)
            {
                RCLCPP_WARN(this->get_logger(), "Threat removal failed! ID is probably incorrect.");
            }
        }
        Threat threat;
        threat.size = request->threat.size;
        threat.intensity = request->threat.intensity;
        cv::Point2f center(request->threat.center.x, request->threat.center.y);
        threat.center = center;
        int offsetX;
        int offsetY;
        auto gaussian = createLocalizedGaussianThreat(threat, offsetX, offsetY);
        threat.offsetX = offsetX + pMap->half_width_diff;
        threat.offsetY = offsetY + pMap->half_height_diff;
        int index = threats.size();
        if (request->id != -1)
        {
            index = request->id;
        }
        try
        {
            RCLCPP_INFO(this->get_logger(), "Setting threat at id: (%d)", index);
            if(int(threats.size())>index){
                threats.at(index) = std::make_pair(threat, gaussian);
                RCLCPP_INFO(this->get_logger(), "Setting directly:");
            } else {
                RCLCPP_INFO(this->get_logger(), "Pushing back:");
                threats.push_back(std::make_pair(threat, gaussian));
                pMap->sampleGaussian(100, threat.center.x+pMap->half_width_diff, threat.center.y+pMap->half_height_diff, 1);
                drawPRM(pMap->PRMMapNodes, pMap->max_dim, pMap->max_dim);
            }
            RCLCPP_INFO(this->get_logger(), "Regenerating threat mask:");
            regenerateThreatMap();
            RCLCPP_INFO(this->get_logger(), "Completed threat mask:");

            response->assigned_id = index;
        }
        catch (const std::out_of_range &e)
        {
            RCLCPP_WARN(this->get_logger(), "SetThreat failed: Do not provide threat IDs for new threats. They will be returned to you.");
        }
        RCLCPP_INFO(this->get_logger(), "Set threat handled");
    }

    void regenerateThreatMap()
    {
        threatsMap = cv::Mat::zeros(pMap->max_dim, pMap->max_dim, CV_32FC1);
        for (auto pair : threats)
        {
            auto threat = pair.first;
            auto gaussian = pair.second;
            applyThreatToMat(threatsMap, gaussian, threat.offsetX, threat.offsetY);
        }
        pMap->apply_threat_mask(threatsMap);
    }

    std::vector<std::pair<double, double>> find_solution(Map &map, double wind_angle_deg, ::MapNode *start_node, ::MapNode *goal_node, uint8_t pathfinding_strategy)
    {
        cv::Mat mat = cv::Mat(map.max_dim, map.max_dim, CV_32FC1, map.data->data());
        cv::Mat scaledImage;
        mat.convertTo(scaledImage, CV_8UC1, 255.0);
        cv::Mat colorImage;
        cv::cvtColor(scaledImage, colorImage, cv::COLOR_GRAY2BGR);
        cv::Scalar redColor(0, 0, 255);
        cv::Scalar greenColor(0, 255, 0);

        cv::circle(colorImage, cv::Point(start_node->x, start_node->y), 1, greenColor, -1);
        cv::circle(colorImage, cv::Point(goal_node->x, goal_node->y), 1, redColor, -1);
        cv::flip(colorImage, colorImage, 0);
        cv::imwrite("/home/sailbot/map_with_points.jpg", colorImage);

        double wind_angle_rad = wind_angle_deg * (M_PI / 180);
        double nogo_angle_rad = NOGO_ANGLE_DEGREES * (M_PI / 180);
        bool wind_blocked = false;
        // start with linear solver
        if (!is_in_nogo(start_node, goal_node, wind_angle_rad, nogo_angle_rad))
        {
            LinearRaycastPathfindingStrategy linearSolver;
            RCLCPP_INFO(this->get_logger(), "Trying linear raycast");

            auto path = linearSolver.solve(map, start_node, goal_node, wind_angle_rad, nogo_angle_rad);
            // return path;
            if (path.size() > 0)
            {
                return path;
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Path is upwind");
            wind_blocked = true;
        }
        // if that fails, try one tack
        if (wind_blocked)
        {
            RCLCPP_INFO(this->get_logger(), "Trying one tack");
            OneTackPathfindingStrategy oneTackSolver;
            auto path = oneTackSolver.solve(map, start_node, goal_node, wind_angle_rad, nogo_angle_rad);
            if (path.size() > 0)
            {
                return path;
            }
        }
        // if both fail, fall back to pathfinding
        RCLCPP_INFO(this->get_logger(), "Falling back to A-star");
        // create solver and solve
        
        PathfindingStrategyBase* solver = nullptr;
        std::vector<std::pair<double, double>> path;
        
        auto time_start = std::chrono::high_resolution_clock::now();
        
        switch(pathfinding_strategy){
            case sailbot_msgs::srv::GetPath::Request::PATHFINDING_STRATEGY_ASTAR:{
                    AStarPathfindingStrategy solver;
                    path = solver.solve(map, start_node, goal_node, wind_angle_rad, nogo_angle_rad);
                    RCLCPP_INFO(this->get_logger(), "Got A-star path");
                }
                break;
            case sailbot_msgs::srv::GetPath::Request::PATHFINDING_STRATEGY_PRMSTAR:{
                    PRMPathfindingStrategy solver;
                    path = solver.solve(map, start_node, goal_node, wind_angle_rad, nogo_angle_rad);
                    RCLCPP_INFO(this->get_logger(), "Got PRM-star path");
                }
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Invalid pathfinding strategy id");
        }

        path = simplify_path(path, wind_angle_deg, map);
        auto time_stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(time_stop - time_start);
        std::string time_string = "Search time: " + std::to_string(duration.count());
        RCLCPP_INFO(this->get_logger(), time_string.c_str());

        if (path.size() == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Path length is zero!");
            std::string start_string = "start: " + std::to_string(start_node->x - map.half_width_diff) + ", " + std::to_string(start_node->y - map.half_height_diff);
            std::string goal_string = "goal: " + std::to_string(goal_node->x - map.half_width_diff) + ", " + std::to_string(goal_node->y - map.half_height_diff);
            RCLCPP_INFO(this->get_logger(), start_string.c_str());
            RCLCPP_INFO(this->get_logger(), goal_string.c_str());
        }
        delete(solver);
        return path;
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