#pragma once
#include <utility>
#include <queue>
#include <math.h>

namespace Sailbot
{
    class Node
    {
    public:
        int x, y;
        float gCost, hCost, fCost;
        Node *parent;
        std::vector<std::pair<int, int>> neighbors;
        Node(int x, int y) : x(x), y(y), gCost(INFINITY), hCost(INFINITY), fCost(INFINITY), parent(nullptr) {}
        Node() : gCost(INFINITY), hCost(INFINITY), fCost(INFINITY), parent(nullptr) {}

        void reset()
        {
            gCost = INFINITY;
            hCost = INFINITY;
            fCost = INFINITY;
            parent = nullptr;
        }
        void calculateFCost()
        {
            fCost = gCost + hCost;
        }
    };

    struct CompareNode
    {
        bool operator()(Node *a, Node *b)
        {
            return a->fCost > b->fCost;
        }
    };
}