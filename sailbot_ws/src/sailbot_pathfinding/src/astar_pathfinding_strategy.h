#pragma once
#include "pathfinding_strategy_base.h"
#define TURN_WEIGHT 1

class AStarPathfindingStrategy : public PathfindingStrategyBase {
public:
	std::vector<std::pair<double, double>> solve(Sailbot::Map& map, Sailbot::Node* start, Sailbot::Node* goal, double wind_angle_rad = 0, double no_go_angle_rad = 0);
private:
	std::vector<Sailbot::Node*> AStar(Sailbot::Map& map, Sailbot::Node* start, Sailbot::Node* goal);
	float turn_penalty(Sailbot::Node* previous, Sailbot::Node* current, Sailbot::Node* next);
	float heuristic(Sailbot::Node* a, Sailbot::Node* b);
};