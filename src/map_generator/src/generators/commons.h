#pragma once
#include <Eigen/Eigen>
#include <vector>
#include <iostream>

using Vec2f = Eigen::Vector2f;
using Vec3f = Eigen::Vector3f;

inline float normalize_theta(const float &angle)
{
    if (angle > -M_PI && angle <= M_PI)
        return angle;
    float result = fmod(angle + M_PI, 2.0f * M_PI);
    if (result <= 0)
        return result + M_PI;
    return result - M_PI;
}
std::vector<std::pair<int, int>> draw_line(const float &x0f, const float &y0f, const float &x1f, const float &y1f);