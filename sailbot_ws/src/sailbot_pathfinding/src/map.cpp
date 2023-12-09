#include "map.h"
#include <vector>
#include <random>
#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "raycast.h"
namespace Sailbot {
    Map::Map(uint32_t map_width, uint32_t map_height) {

        height = map_height;
        width = map_width;

        //calculate maximum integer dimension
        max_dim = ceil(sqrt(pow(height, 2) + pow(width, 2)));

        data = std::make_shared<std::vector<float>>(max_dim * max_dim, 1.0f); // Initialize grid with ones

        //fill interior area with 0
        half_height_diff = (max_dim - height) / 2;
        half_width_diff = (max_dim - width) / 2;

        for (uint32_t y = half_height_diff; y < height + half_height_diff; y++)
            for (uint32_t x = half_width_diff; x < width + half_width_diff; x++)
                data->at(y * max_dim + x) = 0.0;

        neighbors_grid = std::make_shared<std::vector<std::vector<Node>>>();
        neighbors_grid->resize(max_dim, std::vector<Node>(max_dim));

        // Initialize nodes and their neighbors
        for (int y = 0; y < max_dim; ++y) {
            for (int x = 0; x < max_dim; ++x) {
                neighbors_grid->at(y).at(x) = Node(x, y);
                addNeighbors(x, y);
            }
        }

        PRMNodes = std::make_shared<std::vector<std::shared_ptr<Node>>>();
    }

    Map::Map(uint32_t size, std::shared_ptr<std::vector<float>> new_data, std::shared_ptr<std::vector<std::vector<Node>>> new_grid, std::shared_ptr<std::vector<std::shared_ptr<Node>>> new_prm_nodes) {
        max_dim = size;
        data = new_data;
        neighbors_grid = new_grid;
        PRMNodes = new_prm_nodes;
    }


    void Map::addNeighbors(int x, int y) {
        std::vector<std::pair<int, int>> neighborOffsets = { {1, 0}, {0, 1}, {-1, 0}, /*{0, -1},*/ {1, 1}, {1, -1}, {-1, -1}, {-1, 1}}; // 8-directional

        for (auto& offset : neighborOffsets) {
            int nx = x + offset.first;
            int ny = y + offset.second;

            if (nx >= 0 && nx < max_dim && ny >= 0 && ny < max_dim) {
                neighbors_grid->at(y).at(x).neighbors.push_back(&neighbors_grid->at(ny).at(nx));
            }
        }
    }

    Node* Map::getNode(int x, int y) {
        return &neighbors_grid->at(y).at(x);
    }

    Node* Map::randomNode() {
        std::random_device rd;
        std::mt19937 eng(rd());
        std::uniform_int_distribution<> distrX(0, width - 1);
        std::uniform_int_distribution<> distrY(0, height - 1);

        return getNode(distrX(eng)+half_width_diff, distrY(eng)+half_height_diff);
    }

    void Map::generate_obstacles(int num_obstacles, int max_blob_size) {
        std::random_device rd; // Obtain a random number from hardware
        std::mt19937 eng(rd()); // Seed the generator
        std::uniform_int_distribution<> distrX(0, width - 1); // Define range for width
        std::uniform_int_distribution<> distrY(0, height - 1); // Define range for height
        std::uniform_int_distribution<> distrSize(1, max_blob_size); // Define range for blob size

        for (int i = 0; i < num_obstacles; ++i) {
            int blob_start_x = distrX(eng)+half_width_diff;
            int blob_start_y = distrY(eng)+half_height_diff;
            int blob_size = distrSize(eng);

            create_blob(data, blob_start_x, blob_start_y, blob_size);
        }

    }
    void Map::create_blob(std::shared_ptr<std::vector<float>> grid, uint32_t blob_start_x, uint32_t blob_start_y, int blob_size) {
        std::vector<std::pair<int, int>> directions = { {1, 0}, {0, 1}, {-1, 0}, {0, -1} };

        std::random_device rd;
        std::mt19937 eng(rd());
        std::uniform_int_distribution<> distr(0, directions.size() - 1);

        for (uint32_t i = 0; i < blob_size; ++i) {
            uint32_t index = blob_start_y * max_dim + blob_start_x;
            grid->at(index) = 1.0f; // Set the current position to 1

            // Randomly choose a direction
            auto [dx, dy] = directions[distr(eng)];

            // Update start_x and start_y, ensuring they stay within bounds
            blob_start_x = std::max(uint32_t(0), std::min(max_dim - 1, blob_start_x + dx));
            blob_start_y = std::max(uint32_t(0), std::min(static_cast<uint32_t>(grid->size() / max_dim) - 1, blob_start_y + dy));
        }
    }

    Map Map::rotate(double map_angle_deg) {
        //rotate map
        cv::Mat mat = cv::Mat(max_dim, max_dim, CV_32FC1, data->data());
        cv::Point2f center((mat.cols - 1) / 2.0, (mat.rows - 1) / 2.0);    cv::Mat rot = cv::getRotationMatrix2D(center, map_angle_deg, 1.0);

        cv::Mat rotated_mat;
        cv::warpAffine(mat, rotated_mat, rot, mat.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(1.0));

        // Ensure the data type is correct
        if (rotated_mat.type() != CV_32FC1) {
            rotated_mat.convertTo(rotated_mat, CV_32FC1);
        }

        // Flatten the matrix if it's not already a single row or single column
        if (rotated_mat.rows > 1 && rotated_mat.cols > 1) {
            rotated_mat = rotated_mat.reshape(1, 1); // Reshape to a single row
        }

        // Convert cv::Mat to std::vector<float>
        auto rotated_vector = std::make_shared<std::vector<float>>();
        rotated_vector->assign((float*)rotated_mat.datastart, (float*)rotated_mat.dataend);
        Map new_map = Map(max_dim, rotated_vector, neighbors_grid, PRMNodes);
        return new_map;
    }

    bool Map::isWalkable(int x, int y) {
        if (x >= 0 and y >= 0 and x < max_dim and y < max_dim and data->at(gridToIndex(x, y)) < 0.5)
            return true;
        return false;
    }

    bool Map::isBlocked(int x, int y) {
        if (x >= 0 and y >= 0 and x < max_dim and y < max_dim and data->at(gridToIndex(x, y)) < 0.5)
            return false;
        return true;
    }
    int Map::gridToIndex(uint32_t x, uint32_t y) {
        return y * max_dim + x;
    }
}