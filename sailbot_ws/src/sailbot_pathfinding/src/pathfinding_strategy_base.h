#pragma once
#include <vector>
#include "map.h"
#include "node.h"
#define PATHFINDING_BLOCKED_CUTOFF 1.0
class PathfindingStrategyBase {
	virtual std::vector<std::pair<double, double>> solve(Map& map, MapNode* start, MapNode* goal, double wind_angle_rad, double no_go_angle) = 0;
};