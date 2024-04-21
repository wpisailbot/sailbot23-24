#pragma once
#include "pathfinding_strategy_base.h"
#define TURN_WEIGHT 1

class AStarPathfindingStrategy : public PathfindingStrategyBase {
public:
	std::vector<std::pair<double, double>> solve(Map& map, MapNode* start, MapNode* goal, double wind_angle_rad = 0, double no_go_angle_rad = 0);
private:
	std::vector<MapNode*> AStar(Map& map, MapNode* start, MapNode* goal);
	float turn_penalty(MapNode* previous, MapNode* current, MapNode* next);
	float heuristic(MapNode* a, MapNode* b);
};