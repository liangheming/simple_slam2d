#pragma once

#include "commons.h"
#include <limits>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct CloudGeneratorConfig
{
    float translation_thresh{0.1f};
    float rotation_thresh{0.1f};
};
struct CloudGeneratorState
{
    Vec2f min_point{std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
    Vec2f max_point{std::numeric_limits<float>::min(), std::numeric_limits<float>::min()};
};
class CloudGenerator
{
public:
    CloudGenerator(const CloudGeneratorConfig &config) : m_config(config)
    {
        m_cloud.reserve(1000000);
        m_poses.reserve(10000);
        m_indices.reserve(10000);
    }

    void addCloud(const Vec3f &pose, const std::vector<Vec2f> &cloud);
    std::vector<Vec2f> &getCloud() { return m_cloud; }

    void saveCloud(const std::string &filename);

private:
    CloudGeneratorConfig m_config;
    CloudGeneratorState m_state;
    std::vector<Vec2f> m_cloud;
    std::vector<Vec3f> m_poses;
    std::vector<unsigned int> m_indices;
};