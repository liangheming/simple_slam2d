#include "plicp.h"

PLICP::PLICP(const PLICPConfig &config) : m_config(config), m_target_tree(2, m_target_cloud) {}
void PLICP::setTarget(const std::vector<Vec2f> &target)
{
    std::vector<Vec2f> cloud = grid_downsample(m_config.resolution, target);
    m_target_cloud.points.assign(cloud.begin(), cloud.end());
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

        if (valid_count < 10.0f)
            return false;

        delta = Hess.inverse() * lb;
        // std::cout << pose.transpose() << "|" << delta.transpose() << std::endl;
        pose += delta;
        if (delta.cwiseAbs().maxCoeff() < 0.001f)
            break;
    }

    score = total_loss / valid_count;
    // pose.z() = normalize_theta(pose.z());
    // std::cout << "score: " << score << " total_loss:" << total_loss << " valid_count:" << valid_count << " pose: " << pose.transpose() << std::endl;
    // std::cout << "===========================" << std::endl;
    return score < m_config.score_thresh;
}