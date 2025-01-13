#pragma once
#include <cstdint>
#include "commons.h"
#include <unordered_map>
#include <unordered_set>
#include "ieskf2d.h"

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
    VoxelMapBuilder2D() : m_status(0)
    {
        voxel_map.reserve(500000);
        m_cached_imus.clear();
    }
    VoxelMapBuilder2D(const Config &config) : m_config(config), m_status(0)
    {
        voxel_map.reserve(500000);
        m_cached_imus.clear();
    }

    void update(SyncPack &pack);

    VoxelMap2D voxel_map;

    IESKF2D isefk2d;

private:
    VoxelKey2D index(const Vec2f &point);

    bool addPoint(const VoxelKey2D &key, const Vec2f &point);

    bool initIMU(const SyncPack &pack);

    void updatePlane(VoxelGrid2D *grid);

    void updatePlane(const VoxelKey2D &key);

    void addClouds(const Vec3f &pose, const std::vector<Vec2f> &cloud);
    void setConfig(const Config &config) { m_config = config; }

    void propagation(const SyncPack &pack);

    void optimize(SyncPack &pack);

    Config m_config;
    int m_status{0};
    IMUData m_last_imu;
    double m_last_scan_time{0.0};
    std::vector<IMUData> m_cached_imus;
    std::list<VoxelKey2D> m_cache;
};
