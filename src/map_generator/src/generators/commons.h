#pragma once
#include <Eigen/Eigen>

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