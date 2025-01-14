#include "cloud_generator.h"

void CloudGenerator::addCloud(const Vec3f &pose, const std::vector<Vec2f> &cloud)
{
    if (!m_poses.empty())
    {
        Vec3f last_pose = m_poses.back();
        if ((pose - last_pose).head<2>().norm() < m_config.translation_thresh && fabs(normalize_theta(pose.z() - last_pose.z())) < m_config.rotation_thresh)
            return;
    }
    Eigen::Affine2f trans = Eigen::Translation2f(pose.x(), pose.y()) * Eigen::Rotation2Df(pose.z());
    for (const auto &point : cloud)
    {
        Vec2f p = trans * point;
        if (p.x() > m_state.max_point.x())
            m_state.max_point.x() = p.x();
        if (p.y() > m_state.max_point.y())
            m_state.max_point.y() = p.y();
        if (p.x() < m_state.min_point.x())
            m_state.min_point.x() = p.x();
        if (p.y() < m_state.min_point.y())
            m_state.min_point.y() = p.y();
        m_cloud.push_back(p);
    }
    m_poses.push_back(pose);
    m_indices.push_back(m_cloud.size());
}

void CloudGenerator::saveCloud(const std::string &filename)
{
    if (m_cloud.empty())
        return;
    pcl::PCDWriter writer;
    pcl::PointCloud<pcl::PointXYZ> pcl_clouds;
    pcl_clouds.reserve(m_cloud.size());
    for (const auto &point : m_cloud)
        pcl_clouds.emplace_back(pcl::PointXYZ{point.x(), point.y(), 0.0});
    writer.writeBinaryCompressed(filename, pcl_clouds);
}