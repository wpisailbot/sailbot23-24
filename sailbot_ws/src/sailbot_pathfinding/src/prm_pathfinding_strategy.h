#pragma once
#include "pathfinding_strategy_base.h"
#define PRM_TURN_WEIGHT 10
class PRMPathfindingStrategy : public PathfindingStrategyBase {
public:
	std::vector<std::pair<double, double>> solve(Map& map, MapNode* start, MapNode* goal, double wind_angle_rad = 0, double no_go_angle_rad = 0);
private:
	std::vector<std::pair<int, int>> visitedCells;
	std::vector<MapNode*> AStar(Map& map, MapNode* start, MapNode* goal, double wind_angle_rad, double no_go_angle_rad);
	float heuristic(MapNode* a, MapNode* b);
	float turn_penalty(MapNode* previous, MapNode* current, MapNode* next);
};