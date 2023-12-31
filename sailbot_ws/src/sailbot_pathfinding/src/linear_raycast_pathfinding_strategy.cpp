#include "linear_raycast_pathfinding_strategy.h"
#include <iostream>
#define _USE_MATH_DEFINES
#include "math.h"
#include "raycast.h"
#include "utilities.h"

std::vector<std::pair<double, double>> LinearRaycastPathfindingStrategy::solve(Sailbot::Map& map, Sailbot::Node* start, Sailbot::Node* goal, [[maybe_unused]] double wind_angle_rad, [[maybe_unused]] double no_go_angle_rad) {
	if (raycast(map, start->x, start->y, goal->x, goal->y)) {
		return path_to_doubles({ start, goal });
	}
	return {};
}