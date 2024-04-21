#include "prm_pathfinding_strategy.h"
#include <unordered_set>
#include "utilities.h"
float PRMPathfindingStrategy::heuristic(MapNode* a, MapNode* b) {
	return std::hypot(a->x - b->x, a->y - b->y);
}

float PRMPathfindingStrategy::turn_penalty(MapNode* previous, MapNode* current, MapNode* next) {
	// Calculate vectors from previous to current and from current to next
	float dx1 = current->x - previous->x;
	float dy1 = current->y - previous->y;
	float dx2 = next->x - current->x;
	float dy2 = next->y - current->y;

	// Calculate the dot product of the vectors
	float dotProduct = dx1 * dx2 + dy1 * dy2;

	// Calculate the magnitudes of the vectors
	float magnitude1 = sqrt(dx1 * dx1 + dy1 * dy1);
	float magnitude2 = sqrt(dx2 * dx2 + dy2 * dy2);

	// Calculate the cosine of the angle between the vectors
	float cosTheta = dotProduct / (magnitude1 * magnitude2);

	// Clamp the cosine value to the range [-1,1] to avoid any precision issues
	cosTheta = std::max(-1.0f, std::min(1.0f, cosTheta));

	// Calculate the actual angle in radians
	float angle = acos(cosTheta);

	// float angle_degrees = angle * 180.0 / M_PI;

	return angle * TURN_WEIGHT; // Scale the penalty by the angle
}

std::vector<MapNode*> PRMPathfindingStrategy::AStar(Map& map, MapNode* start, MapNode* goal, double wind_angle_rad, double no_go_angle_rad) {
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
			return path;
		}

		closedSet.insert(currentMapNode);
		for (MapNode* neighbor : currentMapNode->neighbors) {
			if (closedSet.contains(neighbor) || !map.isWalkable(neighbor->x, neighbor->y) || is_in_nogo(currentMapNode, neighbor, wind_angle_rad, no_go_angle_rad)) {
				continue;
			}
			float currentTurnPenalty = 0;
			if (currentMapNode->parent != nullptr) {
				currentTurnPenalty = turn_penalty(currentMapNode->parent, currentMapNode, neighbor);
			}
			float tentativeGCost = currentMapNode->gCost + heuristic(currentMapNode, neighbor)+currentTurnPenalty;
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

std::vector<std::pair<double, double>> PRMPathfindingStrategy::solve(Map& map, MapNode* start, MapNode* goal, double wind_angle_rad, double no_go_angle_rad) {
	MapNode* prmStart = map.addSinglePRMMapNode(start->x, start->y, map.prm_connection_radius);
	MapNode* prmGoal = map.addSinglePRMMapNode(goal->x, goal->y, map.prm_connection_radius);
	auto path = AStar(map, prmStart, prmGoal, wind_angle_rad, no_go_angle_rad);
	return path_to_doubles(path);
}