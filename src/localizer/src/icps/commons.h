#pragma once
#include <Eigen/Eigen>
#include <limits>
#include <unordered_map>

#define HASH_P 116101
#define MAX_N 10000000000

using Vec2f = Eigen::Vector2f;
using Vec3f = Eigen::Vector3f;
using Vec2i = Eigen::Vector2i;
using Mat2f = Eigen::Matrix2f;
using Mat3f = Eigen::Matrix3f;
inline float normalize_theta(const float &angle)
{
    if (angle > -M_PI && angle <= M_PI)
        return angle;
    float result = fmod(angle + M_PI, 2.0f * M_PI);
    if (result <= 0)
        return result + M_PI;
    return result - M_PI;
}

struct VoxelKey2D
{
    int64_t x, y;
    VoxelKey2D() : x(0), y(0) {}
    VoxelKey2D(const int64_t &x_in, const int64_t &y_in) : x(x_in), y(y_in) {}
    bool operator==(const VoxelKey2D &other) const
    {
        return (x == other.x && y == other.y);
    }
};
struct VoxelKeyHash2D
{
    int64_t operator()(const VoxelKey2D &k) const
    {
        return (k.y * HASH_P) % MAX_N + k.x;
    }
};
inline std::vector<Vec2f> grid_downsample(const float &resolution, const std::vector<Vec2f> &points)
{
    std::vector<Vec2f> ret;
    if (points.empty())
        return ret;
    std::unordered_map<VoxelKey2D, std::pair<float, Vec2f>, VoxelKeyHash2D> grid_cloud;
    grid_cloud.reserve(10000);
    for (const auto &point : points)
    {
        Vec2i xy = (point / resolution).array().floor().cast<int>();
        VoxelKey2D key(xy.x(), xy.y());
        auto iter = grid_cloud.find(key);
        std::pair<float, Vec2f> *pair_ptr;
        if (iter == grid_cloud.end())
            pair_ptr = &(grid_cloud.insert({key, std::make_pair(0.0f, Vec2f::Zero())}).first->second);
        else
            pair_ptr = &(iter->second);
        pair_ptr->first++;
        pair_ptr->second += point;
    }
    ret.reserve(grid_cloud.size());
    for (auto &pair : grid_cloud)
        ret.push_back(pair.second.second / pair.second.first);
    return ret;
}

inline void affine2Vec(const Eigen::Affine2f &aff, Vec3f &pose)
{
    pose << aff.translation().x(), aff.translation().y(), atan2(aff.rotation().coeff(1, 0), aff.rotation().coeff(0, 0));
}