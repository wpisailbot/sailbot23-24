#pragma once
#include <vector>
#include "map.h"
#include "node.h"
class PathfindingStrategyBase {
	virtual std::vector<std::pair<double, double>> solve(Sailbot::Map& map, Sailbot::Node* start, Sailbot::Node* goal, double wind_angle_rad, double no_go_angle) = 0;
};