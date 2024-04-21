#pragma once
#include <utility>
#include <queue>

class MapNode {
public:
    float x, y;
    float gCost, hCost, fCost;
    MapNode* parent;
    std::vector<MapNode*> neighbors;
    MapNode(float x, float y) : x(x), y(y), gCost(std::numeric_limits<float>::infinity()), hCost(std::numeric_limits<float>::infinity()), fCost(std::numeric_limits<float>::infinity()), parent(nullptr) {}
    MapNode() :gCost(std::numeric_limits<float>::infinity()), hCost(std::numeric_limits<float>::infinity()), fCost(std::numeric_limits<float>::infinity()), parent(nullptr) {}
    void reset()
    {
        gCost = std::numeric_limits<float>::infinity();
        hCost = std::numeric_limits<float>::infinity();
        fCost = std::numeric_limits<float>::infinity();
        parent = nullptr;
    }
    void calculateFCost() {
        fCost = gCost + hCost;
    }
    bool operator == (const MapNode& n) {
        if (x == n.x && y == n.y) {
            return true;
        }
        return false;
    }
};

struct PointKey {
    float x, y;

    bool operator==(const PointKey& other) const {
        return x == other.x && y == other.y;
    }
};

namespace std {
    template <>
    struct hash<PointKey> {
        std::size_t operator()(const PointKey& k) const {
            return ((std::hash<float>()(k.x) ^ (std::hash<float>()(k.y) << 1)) >> 1);
        }
    };
}

struct CompareMapNode {
    bool operator()(MapNode* a, MapNode* b) {
        return a->fCost > b->fCost;
    }
};