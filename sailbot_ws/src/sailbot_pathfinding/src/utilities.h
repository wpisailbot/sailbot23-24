#pragma once
#include <cmath>
#include <memory>
#include "node.h"
#include "map.h"
#include <opencv2/opencv.hpp>

#define NOGO_ANGLE_DEGREES 45

struct Threat {
    cv::Point2f center;  // Using Point2f for subpixel precision
    int offsetX;
    int offsetY;
    float size;
    float intensity;
};

double double_equals(double x, double y, double absTol = 0.000000001, double relTol = 0.000000001);
double distance(double x1, double y1, double x2, double y2);
std::pair<double, double> rotateAndScale(Sailbot::Node* pt, double radians, uint32_t h, uint32_t w, uint32_t h_new, uint32_t w_new);
std::vector<std::pair<double, double>> rotate_path_doubles(std::vector<Sailbot::Node*> path, uint32_t oldHeight, uint32_t oldWidth, uint32_t newHeight, uint32_t newWidth, double angle_deg);
std::vector<std::pair<double, double>> path_to_doubles(std::vector<Sailbot::Node*> path);
std::vector<std::pair<double, double>> simplify_path(std::vector<std::pair<double, double>> originalPath, double wind_angle_deg, Sailbot::Map& map);
int randomAngleDeg();
bool is_in_nogo(Sailbot::Node* start, Sailbot::Node* goal, float wind_angle_rad, float nogo_angle_rad);
bool is_in_nogo(std::pair<double, double> start, std::pair<double, double> goal, float wind_angle_rad, float nogo_angle_rad);
void displayGrid(std::shared_ptr<std::vector<float>> grid, int width, int height, const std::vector<std::pair<double, double>>& path, float windAngle, const char* name);
cv::Mat createLocalizedGaussianThreat(const Threat& threat, int& offsetX, int& offsetY);
void applyThreatToMat(cv::Mat& mat, const cv::Mat& threatMask, int offsetX, int offsetY);