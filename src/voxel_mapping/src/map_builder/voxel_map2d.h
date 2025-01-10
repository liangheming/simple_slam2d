#pragma once
#include <cstdint>
#include "commons.h"
#include <unordered_map>
#include <unordered_set>

#define HASH_P 116101
#define MAX_N 10000000000
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
struct VoxelGrid2D
{
    Vec2f mean{Vec2f::Zero()};
    Vec2f norm{Vec2f::Zero()};
    Mat2f ppt{Mat2f::Zero()};
    float count{0};
    bool is_plane{false};
    std::list<VoxelKey2D>::iterator position_iter{nullptr};
};
using VoxelMap2D = std::unordered_map<VoxelKey2D, VoxelGrid2D, VoxelKeyHash2D>;

class VoxelMapBuilder2D
{
public:
    VoxelMapBuilder2D() { voxel_map.reserve(500000); }
    VoxelMapBuilder2D(const Config &config) : m_config(config) { voxel_map.reserve(500000); }

    void update(ScanPack &package);

    bool optimize(ScanPack &package);

    VoxelKey2D index(const Vec2f &point);

    bool addPoint(const VoxelKey2D &key, const Vec2f &point);

    void updatePlane(VoxelGrid2D *grid);

    void updatePlane(const VoxelKey2D &key);

    void addClouds(const Vec3f &pose, const std::vector<Vec2f> &cloud);
    void setConfig(const Config &config) { m_config = config; }

    VoxelMap2D voxel_map;

private:
    Config m_config;
    bool m_is_initialized{false};
    std::list<VoxelKey2D> m_cache;
};
