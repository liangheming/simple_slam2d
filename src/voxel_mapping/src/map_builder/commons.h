#pragma once
#include <cmath>
#include <Eigen/Eigen>
#include <iostream>
using Vec2f = Eigen::Vector2f;
using Vec3f = Eigen::Vector3f;
using Mat2f = Eigen::Matrix2f;
using Mat3f = Eigen::Matrix3f;

struct Config
{
    float plane_threshold{0.1f};
    float resolution{0.05f};
    float update_thresh{5.0f};
    float max_update_thresh{100.0f};
    int max_iteration{10};
    float move_thresh{0.2f};
    float rotate_thresh{0.1f};
    int max_voxel_number{500000};
    int init_imu_num{20};
    float obs_info{1000.0f};
};

struct IMUData
{
    double time{0.0};
    Vec3f acc{Vec3f::Zero()};
    Vec3f gyro{Vec3f::Zero()};
};

struct SyncPack
{
    double scan_time{0.0}; // 这个是scan的最后一帧的时间
    std::vector<Vec2f> scan;
    std::vector<IMUData> imus;
};
inline float normalize_theta(const float &angle)
{
    float result = fmod(angle + M_PI, 2.0f * M_PI);
    if (result <= 0)
        return result + M_PI;
    return result - M_PI;
}

inline float normalize_angle_pos(float angle)
{
    return fmod(fmod(angle, 2.0f * M_PI) + 2.0f * M_PI, 2.0f * M_PI);
}

inline float normalize_angle(float angle)
{
    float a = normalize_angle_pos(angle);
    if (a > M_PI)
    {
        a -= 2.0f * M_PI;
    }
    return a;
}
