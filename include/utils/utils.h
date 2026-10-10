#pragma once

#include <vector>
#include <Eigen/Dense>

#include "simulation/display.h"

double wrap_angle(double angle);
double calculate_mean(const std::vector<double> &dataset);
double calculate_rmse(const std::vector<double> &dataset);

std::vector<Vector2> generate_ellipse(double x, double y,
                                      double sigma_xx, double sigma_yy, double sigma_xy,
                                      int num_points = 50);

std::vector<Vector2> generate_circle(double x, double y, double radius,
                                     int num_points = 50);

std::vector<Vector2> offset_points(const std::vector<Vector2> &points,
                                   const Vector2 &offset);