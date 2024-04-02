#pragma once
#include <vector>
#include <memory>
#include "node.h"
namespace Sailbot {
	class Map {
	public:
		//for initial construction
		Map(uint32_t map_width, uint32_t map_height, std::vector<int8_t> initial_data);

		//for rotation
		Map(uint32_t size, std::shared_ptr<std::vector<float>> new_data, std::shared_ptr<std::vector<std::vector<Node>>> new_grid);
		Map rotate(double map_angle_deg);
		void addNeighbors(uint32_t x, uint32_t y);
		Node* getNode(uint32_t x, uint32_t y);
		void generate_obstacles(uint32_t num_obstacles, uint32_t max_blob_size);
		bool isWalkable(uint32_t x, uint32_t y);
		bool isBlocked(uint32_t x, uint32_t y);
		int gridToIndex(uint32_t x, uint32_t y);
		Node* randomNode();
		//std::vector<Node*> getNeighbors(Node* node);
		uint32_t width;
		uint32_t height;
		uint32_t max_dim;
		uint32_t half_height_diff;
		uint32_t half_width_diff;
		//right now, we retain pointers to elements in this grid. This is not advisable, as if the vector is resized (as you insert more objects)
		//all of the pointers are invalidated. We manually call resize() to reserve as much space as we need in the constructor to work around this.
		//eventually, we should move to arrays instead.
		std::shared_ptr<std::vector<std::vector<Node>>> neighbors_grid;
		std::shared_ptr<std::vector<float>> data;
	private:
		void create_blob(std::shared_ptr<std::vector<float>> grid, uint32_t blob_start_x, uint32_t blob_start_y, uint32_t blob_size);
	};
}