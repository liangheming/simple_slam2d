#include "localizer_ros.h"

LocalizerROS::LocalizerROS(ros::NodeHandle &nh) : m_nh(nh), m_sync(10)
{
    loadParameters();
    initPublishers();
    initSubscribers();
    initServices();

    m_plicp = std::make_unique<PLICP>(m_plicp_config);
    std::vector<Vec2f> points;
    readPCD(m_config.map_pcd_path, points);
    m_plicp->setTarget(points);
    publishMapCloud(ros::Time::now().toSec());
    m_thread = std::thread(&LocalizerROS::mainLoop, this);
    m_tf_timer = m_nh.createTimer(ros::Duration(0.1), &LocalizerROS::sendtf, this);
}
bool LocalizerROS::readPCD(std::string &pcd_path, std::vector<Vec2f> &points)
{
    if (!std::filesystem::exists(pcd_path))
    {
        ROS_WARN("LocalizerROS::readPCD: pcd file not found %s", pcd_path.c_str());
        return false;
    }
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    reader.read(pcd_path, *cloud);
    points.clear();
    points.reserve(cloud->points.size());
    for (auto &point : cloud->points)
        points.emplace_back(point.x, point.y);
    return true;
}
void LocalizerROS::loadParameters()
{
    m_nh.param<std::string>("odom_topic", m_config.odom_topic, "/voxel_mapping/odom");
    m_nh.param<std::string>("point_topic", m_config.point_topic, "/voxel_mapping/points");
    m_nh.param<std::string>("map_pcd_path", m_config.map_pcd_path, "");
    m_nh.param<std::string>("map_frame", m_config.map_frame, "map");
    m_nh.param<std::string>("odom_frame", m_config.odom_frame, "odom");
    m_nh.param<std::string>("body_frame", m_config.body_frame, "scan");

    m_nh.param<float>("localize_hz", m_config.localize_hz, 10.0);
    m_nh.param<int>("max_acc_num", m_config.max_acc_num, 5);
    assert(m_config.localize_hz > 0);
    m_config.localize_duration = static_cast<int>(1.0f / m_config.localize_hz * 1000.0f);
    m_nh.param<float>("scan_resolution", m_config.scan_resolution, 0.05);

    m_nh.param<int>("k_near", m_plicp_config.k_near, 10);
    m_nh.param<float>("map_resolution", m_plicp_config.resolution, 0.1);
    m_nh.param<int>("max_iteration", m_plicp_config.max_iteration, 10);
    m_nh.param<float>("max_search_distance", m_plicp_config.max_search_distance, 0.5f);
    m_nh.param<float>("plane_thresh", m_plicp_config.plane_thresh, 0.01f);
    m_nh.param<float>("score_thresh", m_plicp_config.score_thresh, 0.1f);

    ROS_DEBUG("LocalizerROS::loadParameters");
    ROS_DEBUG("k_near: %d", m_plicp_config.k_near);
    ROS_DEBUG("resolution: %f", m_plicp_config.resolution);
    ROS_DEBUG("max_iteration: %d", m_plicp_config.max_iteration);

    ROS_DEBUG("odom_topic: %s", m_config.odom_topic.c_str());
    ROS_DEBUG("point_topic: %s", m_config.point_topic.c_str());
    ROS_DEBUG("map_pcd_path: %s", m_config.map_pcd_path.c_str());
    ROS_DEBUG("map_frame: %s", m_config.map_frame.c_str());
    ROS_DEBUG("odom_frame: %s", m_config.odom_frame.c_str());
    ROS_DEBUG("body_frame: %s", m_config.body_frame.c_str());
    ROS_DEBUG("localize_hz: %f", m_config.localize_hz);
    ROS_DEBUG("max_acc_num: %d", m_config.max_acc_num);
    ROS_DEBUG("scan_resolution: %f", m_config.scan_resolution);
}
void LocalizerROS::initSubscribers()
{
    m_init_pose_sub = m_nh.subscribe("/initialpose", 1, &LocalizerROS::initPoseCallBack, this);
    m_odom_sub.subscribe(m_nh, m_config.odom_topic, 1);
    m_point_sub.subscribe(m_nh, m_config.point_topic, 1);
    m_sync.connectInput(m_point_sub, m_odom_sub);
    m_sync.registerCallback(boost::bind(&LocalizerROS::syncCallBack, this, _1, _2));
}
void LocalizerROS::initPublishers()
{
    m_map_pub = m_nh.advertise<sensor_msgs::PointCloud>("map", 1, true);
    m_point_pub = m_nh.advertise<sensor_msgs::PointCloud>("points", 1, true);
}
void LocalizerROS::initServices()
{
    m_relocalize_srv = m_nh.advertiseService("relocalize", &LocalizerROS::relocalizeCallBack, this);
}
void LocalizerROS::syncCallBack(const sensor_msgs::PointCloud::ConstPtr &cloud, const nav_msgs::Odometry::ConstPtr &odom)
{
    std::lock_guard<std::mutex> lock(m_state.mutex);
    m_state.queue.emplace(odom, cloud);
    m_state.last_time = odom->header.stamp.toSec();
}
void LocalizerROS::publishMapCloud(const double &timestamp)
{
    if (m_plicp->targetCloud().empty())
        return;
    sensor_msgs::PointCloud msg;
    msg.header.frame_id = m_config.map_frame;
    msg.header.stamp = ros::Time().fromSec(timestamp);

    for (auto &point : m_plicp->targetCloud())
    {
        geometry_msgs::Point32 p;
        p.x = point.x();
        p.y = point.y();
        p.z = 0;
        msg.points.push_back(p);
    }
    m_map_pub.publish(msg);
}

void LocalizerROS::publishMergedCloud(const std::vector<Vec2f> &points, const double &timestamp)
{
    if (m_point_pub.getNumSubscribers() == 0 || points.empty())
        return;
    sensor_msgs::PointCloud msg;
    msg.header.frame_id = m_config.body_frame;
    msg.header.stamp = ros::Time().fromSec(timestamp);
    for (auto &point : points)
    {
        geometry_msgs::Point32 p;
        p.x = point.x();
        p.y = point.y();
        p.z = 0;
        msg.points.push_back(p);
    }
    m_point_pub.publish(msg);
}
void LocalizerROS::mainLoop()
{
    while (ros::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(m_config.localize_duration));
        if (m_state.queue.empty())
            continue;

        int count = std::max(0, int(m_state.queue.size()) - m_config.max_acc_num);
        m_state.mutex.lock();
        for (int i = 0; i < count; i++)
            m_state.queue.pop();
        std::vector<std::pair<nav_msgs::Odometry::ConstPtr, sensor_msgs::PointCloud::ConstPtr>> cache;
        cache.reserve(m_state.queue.size());
        while (!m_state.queue.empty())
        {
            cache.emplace_back(m_state.queue.front());
            m_state.queue.pop();
        }
        m_state.mutex.unlock();

        Eigen::Affine2f last_trans = Eigen::Translation2f(cache.back().first->pose.pose.position.x, cache.back().first->pose.pose.position.y) * Eigen::Rotation2Df(tf::getYaw(cache.back().first->pose.pose.orientation));
        std::vector<Vec2f> points;
        points.reserve(10000);
        for (auto &pack : cache)
        {
            Eigen::Affine2f trans = Eigen::Translation2f(pack.first->pose.pose.position.x, pack.first->pose.pose.position.y) * Eigen::Rotation2Df(tf::getYaw(pack.first->pose.pose.orientation));
            Eigen::Affine2f delta_trans = last_trans.inverse() * trans;
            for (auto &point : pack.second->points)
            {
                Vec2f p(point.x, point.y);
                p = delta_trans * p;
                points.emplace_back(p);
            }
        }
        std::vector<Vec2f> input_points = grid_downsample(m_plicp_config.resolution, points);
        publishMergedCloud(input_points, cache.back().first->header.stamp.toSec());
        m_state.reloc_mutex.lock();
        if (m_state.relocalize_called)
        {
            Eigen::Affine2f mb = Eigen::Translation2f(m_state.reloc_pose.x(), m_state.reloc_pose.y()) * Eigen::Rotation2Df(m_state.reloc_pose.z());
            Eigen::Affine2f diff = mb * last_trans.inverse();
            affine2Vec(diff, m_state.pose_diff);
        }
        m_state.reloc_mutex.unlock();

        Eigen::Affine2f delta_trans = Eigen::Translation2f(m_state.pose_diff.x(), m_state.pose_diff.y()) * Eigen::Rotation2Df(m_state.pose_diff.z());
        Eigen::Affine2f init_trans = delta_trans * last_trans;
        Vec3f init_pose;
        affine2Vec(init_trans, init_pose);
        float score;
        if (m_plicp->align(input_points, score, init_pose))
        {
            m_state.reloc_mutex.lock();
            ROS_DEBUG("LocalizerROS::mainLoop: success");
            ROS_DEBUG("LocalizerROS::mainLoop: refined: %f, %f, %f", init_pose.x(), init_pose.y(), init_pose.z());
            m_state.relocalize_called = false;
            Eigen::Affine2f mb = Eigen::Translation2f(init_pose.x(), init_pose.y()) * Eigen::Rotation2Df(init_pose.z());
            affine2Vec(mb * last_trans.inverse(), m_state.pose_diff);
            ROS_DEBUG("LocalizerROS::mainLoop: diff: %f, %f, %f", m_state.pose_diff.x(), m_state.pose_diff.y(), m_state.pose_diff.z());
            m_state.reloc_mutex.unlock();
        }
    }
}
void LocalizerROS::sendtf(const ros::TimerEvent &)
{
    if (m_state.last_time < 0 || m_state.send_time == m_state.last_time)
        return;

    m_state.reloc_mutex.lock();
    Vec3f diff = m_state.pose_diff;
    m_state.reloc_mutex.unlock();
    geometry_msgs::TransformStamped msg;
    msg.header.frame_id = m_config.map_frame;
    msg.header.stamp = ros::Time().fromSec(m_state.last_time);
    msg.child_frame_id = m_config.odom_frame;
    msg.transform.translation.x = diff.x();
    msg.transform.translation.y = diff.y();
    msg.transform.translation.z = 0;
    msg.transform.rotation = tf::createQuaternionMsgFromYaw(diff.z());
    m_tf_broadcaster.sendTransform(msg);
    m_state.send_time = m_state.last_time;
}

void LocalizerROS::initPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose)
{
    m_state.reloc_mutex.lock();
    m_state.relocalize_called = true;
    m_state.reloc_pose = Vec3f(pose->pose.pose.position.x, pose->pose.pose.position.y, tf::getYaw(pose->pose.pose.orientation));
    m_state.reloc_mutex.unlock();
    ROS_DEBUG("LocalizerROS::reloc_pose: %f %f %f", m_state.reloc_pose.x(), m_state.reloc_pose.y(), m_state.reloc_pose.z());
}

bool LocalizerROS::relocalizeCallBack(interfaces::ReLocalize::Request &req, interfaces::ReLocalize::Response &res)
{
    if (req.reload)
    {
        std::filesystem::path pcd_path(req.pcd_path);
        if (!std::filesystem::exists(pcd_path))
        {
            res.success = false;
            res.message = pcd_path.string() + " is not exist";
            return true;
        }
        m_state.reloc_mutex.lock();
        std::vector<Vec2f> points;
        readPCD(req.pcd_path, points);
        m_plicp->setTarget(points);
        m_state.reloc_mutex.unlock();
    }
    m_state.reloc_mutex.lock();
    m_state.reloc_pose = Vec3f(req.x, req.y, req.yaw);
    m_state.relocalize_called = true;
    m_state.reloc_mutex.unlock();
    res.success = true;
    res.message = "relocalize called";
    return true;
}