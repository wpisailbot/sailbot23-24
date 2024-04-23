#pragma once
#include "map.h"
#define RAYCAST_BLOCKED_CUTOFF 1.0
bool raycast(Map& map, int x1, int y1, int x2, int y2);
float accumulate_danger_raycast(Map& map, int x1, int y1, int x2, int y2);