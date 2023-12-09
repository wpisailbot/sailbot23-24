#pragma once
#include <cmath>
#include <memory>
#include "node.h"

#define M_TAU 2*M_PI

double double_equals(double x, double y, double absTol = 0.000000001, double relTol = 0.000000001);
double distance(double x1, double y1, double x2, double y2);
std::pair<double, double> rotateAndScale(Sailbot::Node* pt, double radians, uint32_t h, uint32_t w, uint32_t h_new, uint32_t w_new);
std::vector<std::pair<double, double>> rotate_path_doubles(std::vector<Sailbot::Node*> path, uint32_t oldHeight, uint32_t oldWidth, uint32_t newHeight, uint32_t newWidth, double angle_deg);
std::vector<std::pair<double, double>> path_to_doubles(std::vector<Sailbot::Node*> path);
bool is_in_nogo(Sailbot::Node* start, Sailbot::Node* goal, float wind_angle_rad, float nogo_angle_rad);