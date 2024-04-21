#include "astar_pathfinding_strategy.h"
#include "utilities.h"
#include <unordered_set>
#include <float.h>
#include <iostream>
#include <chrono>
#include <string>
#define _USE_MATH_DEFINES
#include <math.h>
float AStarPathfindingStrategy::turn_penalty(MapNode* previous, MapNode* current, MapNode* next) {
	if (current->x - previous->x != 0 && next->x - current->x != 0) {
		double slope1 = (current->y - previous->y) / (current->x - previous->x);
		double slope2 = (next->y - current->y) / (next->x - current->x);
		if (double_equals(slope1, slope2)) {
			return 0;
		}
		else {
			return TURN_WEIGHT;
		}
	}
	else {
		if (previous->x == current->x == next->x || previous->y == current->y == next->y) {
			return 0;
		}
		else {
			return TURN_WEIGHT;
		}
	}
}

float AStarPathfindingStrategy::heuristic(MapNode* a, MapNode* b) {
	return std::hypot(a->x - b->x, a->y - b->y);
}

std::vector<MapNode*> AStarPathfindingStrategy::AStar(Map& map, MapNode* start, MapNode* goal) {
	std::priority_queue<MapNode*, std::vector<MapNode*>, CompareMapNode> openSet;
	std::unordered_set<MapNode*> closedSet;
	start->gCost = 0;
	start->hCost = heuristic(start, goal);
	start->calculateFCost();
	openSet.push(start);

	while (!openSet.empty()) {
		MapNode* currentMapNode = openSet.top();
		openSet.pop();
		if (currentMapNode == goal) {
			std::vector<MapNode*> path;
			MapNode* originalCurrent = currentMapNode;
			while (currentMapNode != nullptr) {
				path.push_back(currentMapNode);
				currentMapNode = currentMapNode->parent;
			}
			std::reverse(path.begin(), path.end());
			originalCurrent->reset();
			for (MapNode* node : closedSet) {
				node->reset();
			}
			for (uint32_t i = 0; i < openSet.size(); i++) {
				MapNode* node = openSet.top();
				openSet.pop();
				node->reset();
			}
			//reset nodes
			while (!openSet.empty()) {
				auto node = openSet.top();
				openSet.pop();
				node->gCost = INFINITY;
			}
			for (auto node : closedSet) {
				node->gCost = INFINITY;
			}
			return path;
		}

		closedSet.insert(currentMapNode);
		for (MapNode* neighbor : currentMapNode->neighbors) {
			float x = neighbor->x;
			float y = neighbor->y;
			if (closedSet.contains(neighbor) || !map.isWalkable(x, y)) {
				continue;
			}
			float currentTurnPenalty = 0;
			if (currentMapNode->parent != nullptr) {
				currentTurnPenalty = turn_penalty(currentMapNode->parent, currentMapNode, neighbor);
			}
			float tentativeGCost = currentMapNode->gCost + heuristic(currentMapNode, neighbor) + currentTurnPenalty;
			if (tentativeGCost < neighbor->gCost) {
				neighbor->parent = currentMapNode;
				neighbor->gCost = tentativeGCost;
				neighbor->hCost = heuristic(neighbor, goal);
				neighbor->calculateFCost();
				openSet.push(neighbor);
			}
		}
	}

	return std::vector<MapNode*>(); // Return an empty path if no path is found
}

std::vector<std::pair<double, double>> AStarPathfindingStrategy::solve(Map& map, MapNode* start, MapNode* goal, double wind_angle_rad, double no_go_angle_rad) {
	//rotate map to enable wind restriction
	double map_angle_rad = wind_angle_rad - M_PI / 2;
	double map_angle_deg = map_angle_rad * (180 / M_PI);
	std::cout << "map angle deg:" + std::to_string(map_angle_deg);
	Map* rotated_map = map.rotate(map_angle_deg);

	auto transformed_start_doubles = rotateAndScale(start, map_angle_rad, map.max_dim, map.max_dim, rotated_map->max_dim, rotated_map->max_dim);
	std::pair<uint32_t, uint32_t> transformed_start_cell;
	if (transformed_start_doubles.first > map.max_dim-1) {
		transformed_start_cell.first = map.max_dim-1;
	}
	else if (transformed_start_doubles.first < 0) {
		transformed_start_cell.first = 0;
	}
	else {
		transformed_start_cell.first = uint32_t(transformed_start_doubles.first);
	}
	if (transformed_start_doubles.second > map.max_dim-1) {
		transformed_start_cell.second = map.max_dim-1;
	}
	else if (transformed_start_doubles.second < 0) {
		transformed_start_cell.second = 0;
	}
	else {
		transformed_start_cell.second = uint32_t(transformed_start_doubles.second);
	}
	auto transformed_goal_doubles = rotateAndScale(goal, map_angle_rad, map.max_dim, map.max_dim, rotated_map->max_dim, rotated_map->max_dim);
	std::pair<uint32_t, uint32_t> transformed_goal_cell;
	if (transformed_goal_doubles.first > map.max_dim-1) {
		transformed_goal_cell.first = map.max_dim-1;
	}
	else if (transformed_goal_doubles.first < 0) {
		transformed_goal_cell.first = 0;
	}
	else {
		transformed_goal_cell.first = uint32_t(transformed_goal_doubles.first);
	}
	if (transformed_goal_doubles.second > map.max_dim-1) {
		transformed_goal_cell.second = map.max_dim-1;
	}
	else if (transformed_goal_doubles.second < 0) {
		transformed_goal_cell.second = 0;
	}
	else {
		transformed_goal_cell.second = uint32_t(transformed_goal_doubles.second);
	}

	auto path = AStar(*rotated_map, rotated_map->getMapNode(transformed_start_cell.first, transformed_start_cell.second), rotated_map->getMapNode(transformed_goal_cell.first, transformed_goal_cell.second));
	displayGrid(rotated_map->data, rotated_map->max_dim, rotated_map->max_dim, path_to_doubles(path), 90, "rotated grid");
	delete(rotated_map);
	return rotate_path_doubles(path, map.max_dim, map.max_dim, map.max_dim, map.max_dim, map_angle_deg);
}