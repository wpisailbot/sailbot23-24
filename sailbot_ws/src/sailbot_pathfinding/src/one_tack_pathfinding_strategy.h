#pragma once
#include "pathfinding_strategy_base.h"
class OneTackPathfindingStrategy : public PathfindingStrategyBase {
public:
	std::vector<std::pair<double, double>> solve(Sailbot::Map& map, Sailbot::Node* start, Sailbot::Node* goal, double wind_angle_rad = 0, double no_go_angle_rad = 0);
};