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
void VoxelMapBuilder2D::update(SyncPack &pack)
{
    if (m_status == 0)
    {
        if (initIMU(pack))
            m_status = 1;
    }
    else if (m_status == 1)
    {
        addClouds(Vec3f(isefk2d.X.pos(0), isefk2d.X.pos(1), isefk2d.X.theta), pack.scan);
        m_status = 2;
    }
    else
    {
        optimize(pack);
        addClouds(Vec3f(isefk2d.X.pos(0), isefk2d.X.pos(1), isefk2d.X.theta), pack.scan);
    }
}

bool VoxelMapBuilder2D::initIMU(const SyncPack &pack)
{
    m_cached_imus.insert(m_cached_imus.end(), pack.imus.begin(), pack.imus.end());
    if (m_cached_imus.size() < m_config.init_imu_num)
        return false;
    Vec3f sum = Vec3f::Zero();
    for (auto &imu : m_cached_imus)
    {
        sum.segment<2>(0) += imu.acc.segment<2>(0);
        sum(2) += imu.gyro(2);
    }
    Vec2f mean_acc = sum.segment<2>(0) / m_cached_imus.size();
    float mean_theta = sum(2) / m_cached_imus.size();

    isefk2d.P.setIdentity();
    isefk2d.P.block<3, 3>(5, 5) *= 0.0001;

    isefk2d.Q.setIdentity();
    isefk2d.Q.block<3, 3>(0, 0) *= 0.01;
    isefk2d.Q.block<3, 3>(3, 3) *= 0.0001;

    isefk2d.X.pos.setZero();
    isefk2d.X.vel.setZero();
    isefk2d.X.theta = 0.0;
    isefk2d.X.ba = mean_acc;
    isefk2d.X.bg = mean_theta;
    m_cached_imus.clear();
    m_last_scan_time = pack.scan_time;
    m_last_imu = pack.imus.back();
    return true;
}
void VoxelMapBuilder2D::propagation(const SyncPack &pack)
{
    float dt = 0.0f;
    for (auto &imu : pack.imus)
    {
        dt = (imu.time - m_last_scan_time);
        if (dt > 0.0)
        {
            isefk2d.predict(imu, dt);
            m_last_scan_time = imu.time;
        }
    }
    dt = pack.scan_time - m_last_scan_time;
    isefk2d.predict(pack.imus.back(), dt);
    m_last_scan_time = pack.scan_time;
    m_last_imu = pack.imus.back();
}
void VoxelMapBuilder2D::optimize(SyncPack &package)
{
    propagation(package);
    XState predict_X = isefk2d.X;
    unsigned int valid_count = 0;
    Mat8f Hess;
    Vec8f lb;
    Vec8f delta;

    for (int i = 0; i < m_config.max_iteration; i++)
    {
        Eigen::Affine2f transform = Eigen::Translation2f(isefk2d.X.pos) * Eigen::Rotation2Df(isefk2d.X.theta);
        Hess.setZero();
        lb.setZero();
        valid_count = 0;
        for (unsigned int i = 0; i < package.scan.size(); i++)
        {
            Vec2f point = package.scan[i];
            Vec2f point_world = transform * point;
            VoxelKey2D pid = index(point_world);
            auto iter = voxel_map.find(pid);
            if (iter == voxel_map.end())
                continue;
            if (!iter->second.is_plane)
                continue;
            Mat2x3f dp = Mat2x3f::Zero();
            dp.block<2, 2>(0, 0).setIdentity();
            dp.col(2) = Eigen::Rotation2Df(isefk2d.X.theta + M_PI_2) * point;
            float loss = iter->second.norm.transpose() * (point_world - iter->second.mean);
            Vec3f jacc = dp.transpose() * iter->second.norm;
            Mat3f H_block = jacc * m_config.obs_info * jacc.transpose();
            Vec3f lb_block = -jacc * m_config.obs_info * loss;
            Hess.block<2, 2>(0, 0) += H_block.block<2, 2>(0, 0);
            Hess.block<2, 1>(0, 4) += H_block.block<2, 1>(0, 2);
            Hess.block<1, 2>(4, 0) += H_block.block<1, 2>(2, 0);
            Hess.block<1, 1>(4, 4) += H_block.block<1, 1>(2, 2);
            lb.segment<2>(0) += lb_block.segment<2>(0);
            lb(4) += lb_block(2);
            valid_count++;
        }

        if (valid_count == 0)
        {
            isefk2d.X.theta = normalize_angle(isefk2d.X.theta);
            return;
        }
        Mat8f imu_info = isefk2d.P.inverse();
        Hess += imu_info;
        lb -= (imu_info * (isefk2d.X - predict_X));
        delta = Hess.inverse() * lb;
        isefk2d.X += delta;
        isefk2d.X.theta = normalize_angle(isefk2d.X.theta);
    }
    isefk2d.P = Hess.inverse();
}