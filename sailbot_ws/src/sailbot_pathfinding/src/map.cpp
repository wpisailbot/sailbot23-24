#include "map.h"
#include <vector>
#include <random>
#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "raycast.h"

Map::Map(uint32_t map_width, uint32_t map_height, std::vector<int8_t> initial_data) {

    height = map_height;
    width = map_width;

    //calculate maximum integer dimension
    max_dim = ceil(sqrt(pow(height, 2) + pow(width, 2)));

    data = std::make_shared<std::vector<float>>(max_dim * max_dim, 1.0f); // Initialize grid with ones
    base_data = std::make_shared<std::vector<float>>(max_dim * max_dim, 1.0f); // Initialize grid with ones

    //fill interior area with 0
    half_height_diff = (max_dim - height) / 2;
    half_width_diff = (max_dim - width) / 2;

    int i=0;
    for (uint32_t y = half_height_diff; y < height + half_height_diff; y++){
        for (uint32_t x = half_width_diff; x < width + half_width_diff; x++){
            data->at(y * max_dim + x) = initial_data[i];
            base_data->at(y * max_dim + x) = initial_data[i];
            i++;
        }
    }

    neighbors_grid = std::make_shared<std::vector<std::vector<MapNode>>>();
    neighbors_grid->resize(max_dim, std::vector<MapNode>(max_dim));

    // Initialize nodes and their neighbors
    for (uint32_t y = 0; y < max_dim; ++y) {
        for (uint32_t x = 0; x < max_dim; ++x) {
            neighbors_grid->at(y).at(x) = MapNode(x, y);
            addNeighbors(x, y);
        }
    }

    PRMMapNodes = std::make_shared<std::vector<MapNode*>>();
}

Map::Map(uint32_t size, std::shared_ptr<std::vector<float>> new_data, std::shared_ptr<std::vector<std::vector<MapNode>>> new_grid) {
    max_dim = size;
    data = new_data;
    neighbors_grid = new_grid;
}

Map::~Map() {
    //free allocated PRM nodes
    for (const auto& pair : PRMMapNodeMap) {
        delete pair.second;
    }
}
void Map::addNeighbors(int x, int y) {
    std::vector<std::pair<int, int>> neighborOffsets = { {1, 0}, {0, 1}, {-1, 0}, /*{0, -1},*/ {1, 1}, {1, -1}, {-1, -1}, {-1, 1}}; // 8-directional

    for (auto& offset : neighborOffsets) {
        int nx = x + offset.first;
        int ny = y + offset.second;

        if (nx >= 0 && nx < (int)max_dim && ny >= 0 && ny < (int)max_dim) {
            neighbors_grid->at(y).at(x).neighbors.push_back(&neighbors_grid->at(ny).at(nx));
        }
    }
}

MapNode* Map::getMapNode(int x, int y) {
    return &neighbors_grid->at(y).at(x);
}

std::vector<MapNode*> Map::sampleMapNodes(int numMapNodes) {
    std::vector<MapNode*> nodes;
    for (int i = 0; i < numMapNodes; ++i) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> x_dis(half_width_diff, max_dim-half_width_diff);
        std::uniform_real_distribution<float> y_dis(half_height_diff, max_dim - half_height_diff);
        float rand_x = x_dis(gen);
        float rand_y = y_dis(gen);
        MapNode* node = new MapNode(rand_x, rand_y);
        nodes.push_back(node);
    }
    return nodes;
}

void Map::addPRMMapNodes(std::vector<MapNode*> sampled_nodes) {
    int i = 0;
    // Insert points into the tree
    for (MapNode* node : sampled_nodes) {
        PRMMapNodes->push_back(node);
        PointKey key{ node->x, node->y };
        PRMMapNodeMap[key] = node;
        Point_2 point(node->x, node->y);
        tree.insert(point);
    }

    // Find the nearest neighbors for each point
    for (MapNode* node : sampled_nodes) {
        Point_2 query(node->x, node->y);
        Fuzzy_circle region(query, prm_connection_radius, 0.0 /*exact search*/);
        std::vector<Point_2> result;
        tree.search(std::back_inserter(result), region);
        for (Point_2 found : result) {
            // Convert found Point_2 back to node indices or references
            auto neighborMapNode = findMapNodeByPosition(found.x(), found.y());
            if (neighborMapNode != node) {
                if (raycast(*this, node->x, node->y, neighborMapNode->x, neighborMapNode->y)) {
                    // Connect nodes
                    node->neighbors.push_back(neighborMapNode);
                    //neighborMapNode->neighbors.push_back(node);
                }
            }
        }
    }
}

void Map::sampleGaussian(int numMapNodes, float target_x, float target_y, float std_dev) {
    std::vector<MapNode*> sampled_nodes;
    std::random_device rd;  
    std::mt19937 gen(rd());
    std::normal_distribution<> d_x(target_x, std_dev);
    std::normal_distribution<> d_y(target_y, std_dev);

    for (int i = 0; i < numMapNodes; ++i) {
        float rand_x = d_x(gen);
        float rand_y = d_y(gen);

        // Ensure that the sampled points are within bounds
        rand_x = std::max(0.0f, std::min(rand_x, float(max_dim)));
        rand_y = std::max(0.0f, std::min(rand_y, float(max_dim)));

        MapNode* node = new MapNode(rand_x, rand_y);
        sampled_nodes.push_back(node);
    }
    addPRMMapNodes(sampled_nodes);
}

void Map::initPRM(float num_samples, float connection_radius_percent) {

    // Start with a low-resolution grid
    //int step = 5;
    //float hypot = sqrt(2 * float(pow(step, 2)));
    //for (int y = half_height_diff; y < max_dim - half_height_diff; y += step) {
    //    for (int x = half_width_diff; x < max_dim - half_width_diff; x += step) {
    //        addSinglePRMMapNode(x, y, hypot+0.1);
    //    }
    //}

    //prm_connection_radius = connection_radius;
    //int num_samples = samples_per_unit_squared * ((max_dim - 2 * half_height_diff) * (max_dim - 2 * half_width_diff));
    prm_connection_radius = std::max(width, height) * (connection_radius_percent / 100);
    auto sampled_nodes = sampleMapNodes(num_samples);

    addPRMMapNodes(sampled_nodes);
}

MapNode* Map::findMapNodeByPosition(float x, float y) {
    PointKey key{ x, y };
    auto it = PRMMapNodeMap.find(key);
    if (it != PRMMapNodeMap.end()) {
        return it->second;
    }
    return nullptr; // If no node found
}

MapNode* Map::addSinglePRMMapNode(float x, float y, float connection_radius) {
    int id = PRMMapNodes->size();
    auto new_node = new MapNode(x, y);
    PointKey key{ new_node->x, new_node->y };
    PRMMapNodeMap[key] = new_node;
    Point_2 point(new_node->x, new_node->y);
    tree.insert(point);

    Fuzzy_circle region(point, connection_radius, 0.0 /*exact search*/);
    std::vector<Point_2> result;
    tree.search(std::back_inserter(result), region);

    for (const auto& found : result) {
        // Convert found Point_2 back to node indices or references
        auto neighborMapNode = findMapNodeByPosition(found.x(), found.y());
        if (neighborMapNode != new_node) {
            if (raycast(*this, new_node->x, new_node->y, neighborMapNode->x, neighborMapNode->y)) {
                // Connect nodes
                new_node->neighbors.push_back(neighborMapNode);
                neighborMapNode->neighbors.push_back(new_node);
            }
        }
    }
    PRMMapNodes->push_back(new_node);
    return(new_node);
}

MapNode* Map::randomMapNode() {
    std::random_device rd;
    std::mt19937 eng(rd());
    std::uniform_int_distribution<> distrX(0, width - 1);
    std::uniform_int_distribution<> distrY(0, height - 1);

    return getMapNode(distrX(eng)+half_width_diff, distrY(eng)+half_height_diff);
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
void Map::create_blob(std::shared_ptr<std::vector<float>> grid, uint32_t blob_start_x, uint32_t blob_start_y, uint32_t blob_size) {
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

void Map::apply_threat_mask(cv::Mat threat_mask){

    cv::Mat mat = cv::Mat(max_dim, max_dim, CV_32FC1, base_data->data());
    cv::Mat result;
    cv::add(mat, threat_mask, result);
    std::vector<float> flattened(result.begin<float>(), result.end<float>());
    data = std::make_shared<std::vector<float>>(flattened);
}

Map* Map::rotate(double map_angle_deg) {
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
    return new Map(max_dim, rotated_vector, neighbors_grid);
}

bool Map::isWalkable(float x, float y, float blockedCutoff) {
    if (x >= 0 and y >= 0 and x < max_dim and y < max_dim and data->at(gridToIndex(x, y)) < blockedCutoff)
        return true;
    return false;
}

bool Map::isBlocked(float x, float y, float blockedCutoff) {
    if (x >= 0 and y >= 0 and x < max_dim and y < max_dim and data->at(gridToIndex(x, y)) < blockedCutoff)
        return false;
    return true;
}
int Map::gridToIndex(float x, float y) {
    return (uint)y * max_dim + (uint)x;
}

float Map::valueAt(float x, float y) {
    return data->at(gridToIndex(x, y));
}