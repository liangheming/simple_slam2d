#include "voxel_map2d.h"

VoxelKey2D VoxelMapBuilder2D::index(const Vec2f &point)
{
    auto idx = (point / m_config.resolution).array().floor();
    return VoxelKey2D(static_cast<int64_t>(idx(0)), static_cast<int64_t>(idx(1)));
}

bool VoxelMapBuilder2D::addPoint(const VoxelKey2D &key, const Vec2f &point)
{
    VoxelGrid2D *value;
    auto iter = voxel_map.find(key);
    if (iter == voxel_map.end())
    {
        value = &(voxel_map.insert(std::make_pair(key, VoxelGrid2D())).first->second);
        m_cache.push_front(key);
        value->position_iter = m_cache.begin();
        if (m_cache.size() > m_config.max_voxel_number)
        {
            voxel_map.erase(m_cache.back());
            m_cache.pop_back();
        }
    }
    else
    {
        value = &(iter->second);
        m_cache.splice(m_cache.begin(), m_cache, value->position_iter);
    }

    if (value->count >= m_config.max_update_thresh)
        return false;
    value->mean += (point - value->mean) / (value->count + 1.0f);
    value->ppt += point * point.transpose();
    value->count += 1.0f;
    return true;
}
void VoxelMapBuilder2D::addClouds(const Vec3f &pose, const std::vector<Vec2f> &cloud)
{
    Eigen::Affine2f transform = Eigen::Translation2f(pose.head<2>()) * Eigen::Rotation2Df(pose(2));
    std::unordered_set<VoxelKey2D, VoxelKeyHash2D> visited;
    visited.reserve(cloud.size());
    for (auto &point : cloud)
    {
        Vec2f p = transform * point;
        VoxelKey2D idx = index(p);
        if (addPoint(idx, p))
            visited.insert(idx);
    }
    for (auto &idx : visited)
        updatePlane(idx);
}
bool VoxelMapBuilder2D::optimize(ScanPack &package)
{

    Vec3f pose = package.pose;
    Mat3f Hess;
    Vec3f lb;
    Vec3f delta;

    for (unsigned int i = 0; i < m_config.max_iteration; i++)
    {
        Eigen::Affine2f transform = Eigen::Translation2f(pose.head<2>()) * Eigen::Rotation2Df(pose(2));
        Hess.setZero();
        lb.setZero();
        unsigned int valid_count = 0;
        for (unsigned int i = 0; i < package.points.size(); i++)
        {
            Vec2f point = package.points[i];
            Vec2f point_world = transform * point;
            VoxelKey2D pid = index(point_world);
            auto iter = voxel_map.find(pid);
            if (iter == voxel_map.end())
                continue;

            if (iter->second.is_plane)
            {
                Eigen::Matrix<float, 2, 3> dp;
                dp.block<2, 2>(0, 0).setIdentity();
                dp.col(2) = Vec2f(-sin(pose(2)) * point.x() - cos(pose(2)) * point.y(), cos(pose(2)) * point.x() - sin(pose(2)) * point.y());
                float loss = iter->second.norm.transpose() * (point_world - iter->second.mean);
                Vec3f jacc = dp.transpose() * iter->second.norm;
                Hess += jacc * 1000.0 * jacc.transpose();
                lb -= 1000.0 * jacc * loss;
                valid_count++;
            }
        }
        if (valid_count == 0)
        {
            std::cout << "no valid point" << std::endl;
            return false;
        }

        delta = Hess.inverse() * lb;
        pose += delta;
        if (abs(delta.x()) < 0.001 && abs(delta.y()) < 0.001 && abs(delta.z()) < 0.002)
            break;
    }
    pose(2) = normalize_angle(pose(2));
    package.pose = pose;
    return true;
}
void VoxelMapBuilder2D::update(ScanPack &package)
{
    if (!m_is_initialized)
    {
        m_is_initialized = true;
        addClouds(package.pose, package.points);
        return;
    }
    optimize(package);

    addClouds(package.pose, package.points);
}
void VoxelMapBuilder2D::updatePlane(VoxelGrid2D *grid)
{
    if (grid->count < m_config.update_thresh)
        return;
    if (grid->count > m_config.max_update_thresh)
        return;
    Mat2f cov = grid->ppt / grid->count - grid->mean * grid->mean.transpose();
    Eigen::SelfAdjointEigenSolver<Mat2f> es(cov);
    float eval = es.eigenvalues()(0);
    if (eval > m_config.plane_threshold)
    {
        grid->is_plane = false;
        return;
    }
    grid->is_plane = true;
    grid->norm = es.eigenvectors().col(0);
}
void VoxelMapBuilder2D::updatePlane(const VoxelKey2D &key)
{
    auto iter = voxel_map.find(key);
    if (iter == voxel_map.end())
        return;
    updatePlane(&(iter->second));
}