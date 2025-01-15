#pragma once
#include "commons.h"
#include <limits>
#include "nanoflann.hpp"
#include <chrono>
#include <iostream>

class KDCloud
{
public:
    inline size_t kdtree_get_point_count() const { return points.size(); }
    inline float kdtree_get_pt(const size_t idx, const size_t dim) const { return dim == 0 ? points[idx].x() : points[idx].y(); }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX & /* bb */) const { return false; }
    std::vector<Vec2f> points;
};

using KDTree2D = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, KDCloud>, KDCloud, 2>;

struct PLICPConfig
{
    float resolution{0.1};
    int max_iteration{10};
    int k_near{10};
    float max_search_distance{0.5f};
    float plane_thresh{0.01f};
    bool hybrid{false};
    float score_thresh{0.1f};
};

class PLICP
{
public:
    PLICP(const PLICPConfig &config);

    void setTarget(const std::vector<Vec2f> &target);

    bool align(const std::vector<Vec2f> &input, float &score, Vec3f &pose);
    std::vector<Vec2f> &targetCloud() { return m_target_cloud.points; }

private:
    PLICPConfig m_config;
    KDCloud m_target_cloud;
    KDTree2D m_target_tree;
};