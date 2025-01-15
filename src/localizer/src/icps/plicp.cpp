#include "plicp.h"

PLICP::PLICP(const PLICPConfig &config) : m_config(config), m_target_tree(2, m_target_cloud) {}
void PLICP::setTarget(const std::vector<Vec2f> &target)
{
    Vec2f x0y0(std::numeric_limits<float>::max(), std::numeric_limits<float>::max()), x1y1(std::numeric_limits<float>::min(), std::numeric_limits<float>::min());
    // 找到最小最大点
    for (const auto &point : target)
    {
        x0y0.x() = std::min(x0y0.x(), point.x());
        x0y0.y() = std::min(x0y0.y(), point.y());
        x1y1.x() = std::max(x1y1.x(), point.x());
        x1y1.y() = std::max(x1y1.y(), point.y());
    }
    // 按照栅格进行将采样
    std::unordered_map<unsigned int, std::pair<int, Vec2f>> grid_cloud;
    int width = std::ceil((x1y1.x() - x0y0.x()) / m_config.resolution);
    int height = std::ceil((x1y1.y() - x0y0.y()) / m_config.resolution);
    grid_cloud.reserve(width * height);
    for (const auto &point : target)
    {
        Vec2i xy = ((point - x0y0) / m_config.resolution).array().floor().cast<int>();
        unsigned int idx = xy.y() * width + xy.x();
        auto iter = grid_cloud.find(idx);
        std::pair<int, Vec2f> *pair_ptr;
        if (iter == grid_cloud.end())
            pair_ptr = &(grid_cloud.insert({idx, std::make_pair(0, Vec2f::Zero())}).first->second);
        else
            pair_ptr = &(iter->second);
        pair_ptr->first++;
        pair_ptr->second += point;
    }
    m_target_cloud.points.reserve(grid_cloud.size());
    for (auto &pair : grid_cloud)
        m_target_cloud.points.emplace_back(pair.second.second / static_cast<float>(pair.second.first));
    m_target_tree.buildIndex();
}

bool PLICP::align(const std::vector<Vec2f> &input, float &score, Vec3f &pose)
{
    std::vector<uint32_t> indices(m_config.k_near);
    std::vector<float> distances(m_config.k_near);
    float valid_count, point_count, loss, total_loss;
    float sqr_max_distance = m_config.max_search_distance * m_config.max_search_distance;

    Vec2f norm, mean, nearest_point;
    Mat2f ppt, cov;
    bool is_plane;
    Mat3f Hess;
    Vec3f lb, delta;

    for (int i = 0; i < m_config.max_iteration; i++)
    {
        valid_count = 0.0;
        total_loss = 0.0;
        Hess.setZero();
        lb.setZero();
        Eigen::Affine2f transform = Eigen::Translation2f(pose.x(), pose.y()) * Eigen::Rotation2Df(pose.z());
        for (const auto &point : input)
        {
            Vec2f p = transform * point;
            float query_pt[2] = {p.x(), p.y()};
            indices.clear();
            distances.clear();
            m_target_tree.knnSearch(query_pt, m_config.k_near, indices.data(), distances.data());
            if (distances[0] > sqr_max_distance)
                continue;
            nearest_point = m_target_cloud.points[indices[0]];

            point_count = 0.0;
            mean.setZero();
            ppt.setZero();
            cov.setZero();
            is_plane = false;
            for (int j = 0; j < m_config.k_near; j++)
            {
                if (distances[j] > sqr_max_distance)
                    break;
                Vec2f &pj = m_target_cloud.points[indices[j]];
                mean += (pj - mean) / (point_count + 1.0f);
                ppt += pj * pj.transpose();
                point_count += 1.0f;
            }
            if (point_count < 5.0f)
                continue;
            cov = ppt / point_count - mean * mean.transpose();
            Eigen::SelfAdjointEigenSolver<Mat2f> es(cov);
            is_plane = es.eigenvalues()[0] < m_config.plane_thresh;
            if (!is_plane && !m_config.hybrid)
                continue;
            valid_count += 1.0f;
            norm = es.eigenvectors().col(0);
            loss = (p - mean).transpose() * norm;
            total_loss += abs(loss);
            Vec3f dp(norm.x(), norm.y(), norm.transpose() * (Eigen::Rotation2Df(pose.z() + M_PI_2) * point));
            Hess += dp * 100.0f * dp.transpose();
            lb -= dp * 100.0f * loss;
        }

        if (valid_count < 5.0f)
            return false;

        delta = Hess.inverse() * lb;
        pose += delta;
        if (delta.cwiseAbs().maxCoeff() < 0.001f)
            break;
    }
    score = total_loss / valid_count;
    pose.z() = normalize_theta(pose.z());
    return score < m_config.score_thresh;
}