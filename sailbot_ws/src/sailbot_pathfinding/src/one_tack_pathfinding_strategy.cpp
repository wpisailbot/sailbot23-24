#include "one_tack_pathfinding_strategy.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "raycast.h"
#include "utilities.h"
#include <iostream>
#include <fstream>


std::vector<std::pair<double, double>> OneTackPathfindingStrategy::solve(Map& map, MapNode* start, MapNode* goal, double wind_angle_rad, double no_go_angle_rad) {
	uint32_t h = map.height;
	uint32_t w = map.width;
	double distAtoB = sqrt(pow(goal->x - start->x, 2) + pow(goal->y - start->y, 2));
	double angleAtoB = atan2(goal->y - start->y, goal->x - start->x);
	double angle_nogo_1 = wind_angle_rad - M_PI - no_go_angle_rad;
	double angle_nogo_2 = wind_angle_rad - M_PI + no_go_angle_rad;

	double a = angleAtoB - (wind_angle_rad - M_PI - no_go_angle_rad);
	double b = M_PI - 2 * no_go_angle_rad;
	double c = M_PI - a - b;

	double length1 = (distAtoB * sin(c)) / sin(b);
	double C1_x = int(start->x + length1 * cos(angle_nogo_1));
	double C1_y = int(start->y + length1 * sin(angle_nogo_1));

	double length2 = sqrt(pow(goal->x - C1_x, 2) + pow(goal->y - C1_y, 2));
	double C2_x = int(start->x + length2 * cos(angle_nogo_2));
	double C2_y = int(start->y + length2 * sin(angle_nogo_2));

	float first = 0;
	float second = 0;
	bool first_good = false;
	bool second_good = false;
	if (raycast(map, start->x, start->y, C1_x, C1_y)) {
		first += accumulate_danger_raycast(map, start->x, start->y, C1_x, C1_y);
		if (raycast(map, C1_x, C1_y, goal->x, goal->y)) {
			first += accumulate_danger_raycast(map, C1_x, C1_y, goal->x, goal->y);
			first_good = true;
		}
	}
	if (raycast(map, start->x, start->y, C2_x, C2_y)) {
		second += accumulate_danger_raycast(map, start->x, start->y, C2_x, C2_y);
		if (raycast(map, C2_x, C2_y, goal->x, goal->y)) {
			second += accumulate_danger_raycast(map, C2_x, C2_y, goal->x, goal->y);
			second_good = true;
		}
	}

	if(first_good && second_good){
		if(first < second){
			return path_to_doubles({ start, map.getMapNode(C1_x, C1_y), goal });
		} else {
			return path_to_doubles({ start, map.getMapNode(C2_x, C2_y), goal });
		}
	} else if (first_good){
		return path_to_doubles({ start, map.getMapNode(C1_x, C1_y), goal });
	} else if (second_good) {
		return path_to_doubles({ start, map.getMapNode(C2_x, C2_y), goal });
	}
	return {};
}