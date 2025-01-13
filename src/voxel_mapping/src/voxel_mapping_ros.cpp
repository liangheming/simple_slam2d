#include "voxel_mapping_ros.h"
VoxelMappingROS::VoxelMappingROS(ros::NodeHandle &nh) : m_nh(nh)
{
    loadParameters();
    initPublishers();
    initSubscribers();
    m_main_loop = std::thread(&VoxelMappingROS::mainLoop, this);
    m_map_builder = std::make_shared<VoxelMapBuilder2D>(m_map_config);
}
void VoxelMappingROS::loadParameters()
{
    m_nh.param<std::string>("map_frame", m_config.map_frame, "map");
    m_nh.param<std::string>("body_frame", m_config.body_frame, "body");
    m_nh.param<std::string>("scan_topic", m_config.scan_topic, "/laser/scan");
    m_nh.param<std::string>("imu_topic", m_config.imu_topic, "/laser/imu");
    m_nh.param<float>("ignore_range", m_config.ignore_range, 0.5f);

    m_nh.param<float>("resolution", m_map_config.resolution, 0.05f);
    m_nh.param<float>("update_thresh", m_map_config.update_thresh, 5.0f);
    m_nh.param<float>("max_update_thresh", m_map_config.max_update_thresh, 100.0f);
    m_nh.param<int>("max_iteration", m_map_config.max_iteration, 10);
    m_nh.param<float>("plane_threshold", m_map_config.plane_threshold, 0.1f);
    m_nh.param<float>("rotate_thresh", m_map_config.rotate_thresh, 0.1f);
    m_nh.param<float>("move_thresh", m_map_config.move_thresh, 0.2f);
    m_nh.param<int>("init_imu_num", m_map_config.init_imu_num, 20);
    m_nh.param<int>("max_voxel_number", m_map_config.max_voxel_number, 500000);

    ROS_DEBUG("map_frame: %s", m_config.map_frame.c_str());
    ROS_DEBUG("body_frame: %s", m_config.body_frame.c_str());
    ROS_DEBUG("scan_topic: %s", m_config.scan_topic.c_str());
    ROS_DEBUG("imu_topic: %s", m_config.imu_topic.c_str());
    ROS_DEBUG("ignore_range: %f", m_config.ignore_range);

    ROS_DEBUG("resolution: %f", m_map_config.resolution);
    ROS_DEBUG("update_thresh: %f", m_map_config.update_thresh);
    ROS_DEBUG("max_update_thresh: %f", m_map_config.max_update_thresh);
    ROS_DEBUG("max_iteration: %d", m_map_config.max_iteration);
    ROS_DEBUG("plane_threshold: %f", m_map_config.plane_threshold);
    ROS_DEBUG("rotate_thresh: %f", m_map_config.rotate_thresh);
    ROS_DEBUG("move_thresh: %f", m_map_config.move_thresh);
    ROS_DEBUG("init_imu_num: %d", m_map_config.init_imu_num);
    ROS_DEBUG("max_voxel_number: %d", m_map_config.max_voxel_number);
}
void VoxelMappingROS::initPublishers()
{
    m_scan_pub = m_nh.advertise<sensor_msgs::PointCloud>("points", 1);
    m_odom_pub = m_nh.advertise<nav_msgs::Odometry>("odom", 1);
}
void VoxelMappingROS::initSubscribers()
{
    m_imu_sub = m_nh.subscribe(m_config.imu_topic, 100, &VoxelMappingROS::imuCallback, this);
    m_scan_sub = m_nh.subscribe(m_config.scan_topic, 100, &VoxelMappingROS::scanCallback, this);
}
void VoxelMappingROS::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(m_state.scan_mutex);
    m_state.scan_last_time = msg->header.stamp.toSec();
    m_state.scan_queue.push(msg);
}
void VoxelMappingROS::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(m_state.imu_mutex);
    m_state.imu_last_time = msg->header.stamp.toSec();
    m_state.imu_queue.push(msg);
}

void VoxelMappingROS::filterScan(const sensor_msgs::LaserScan::ConstPtr &msg, std::vector<Vec2f> &scan)
{
    const float &angle_min = msg->angle_min;
    const float &angle_increment = msg->angle_increment;
    float range_min = msg->range_min + 0.01;
    float range_max = msg->range_max - 0.01;
    for (unsigned int i = 0; i < msg->ranges.size(); ++i)
    {
        float range = msg->ranges[i];
        if (std::isinf(range) || std::isnan(range) || range < range_min || range > range_max || range < m_config.ignore_range)
            continue;
        float angle = angle_min + i * angle_increment;
        angle = normalize_theta(angle);
        scan.emplace_back(Vec2f(range * cos(angle), range * sin(angle)));
    }
}
bool VoxelMappingROS::syncScanImu()
{
    if (m_state.imu_queue.empty() || m_state.scan_queue.empty())
        return false;
    if (!m_state.scan_pushed)
    {
        m_sync_pack.scan_time = m_state.scan_queue.front()->header.stamp.toSec();
        m_sync_pack.scan.clear();
        filterScan(m_state.scan_queue.front(), m_sync_pack.scan);
        m_state.scan_pushed = true;
    }
    if (m_state.imu_queue.back()->header.stamp.toSec() < m_sync_pack.scan_time)
        return false;

    m_sync_pack.imus.clear();
    while (!m_state.imu_queue.empty() && m_state.imu_queue.front()->header.stamp.toSec() < m_sync_pack.scan_time)
    {
        m_sync_pack.imus.push_back(IMUData());
        const sensor_msgs::Imu::ConstPtr &imu_msg = m_state.imu_queue.front();
        m_sync_pack.imus.back().time = imu_msg->header.stamp.toSec();
        m_sync_pack.imus.back().acc = Vec3f(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
        m_sync_pack.imus.back().gyro = Vec3f(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
        m_state.imu_mutex.lock();
        m_state.imu_queue.pop();
        m_state.imu_mutex.unlock();
    }
    m_state.scan_mutex.lock();
    m_state.scan_queue.pop();
    m_state.scan_mutex.unlock();
    m_state.scan_pushed = false;
    return !m_sync_pack.imus.empty();
}
void VoxelMappingROS::mainLoop()
{
    while (ros::ok())
    {
        m_state.system_mutex.lock();
        bool activate = m_state.is_activated;
        m_state.system_mutex.unlock();
        if (!activate)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (!syncScanImu())
            continue;
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        m_map_builder->update(m_sync_pack);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        ROS_DEBUG("update time: %f ms", std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000);
        publishScanPoints();
        sendTransform();
        publishOdometry();
    }
}
void VoxelMappingROS::publishScanPoints()
{
    if (m_sync_pack.scan.empty() || m_scan_pub.getNumSubscribers() == 0)
        return;
    sensor_msgs::PointCloud points;
    points.header.frame_id = m_config.body_frame;
    points.header.stamp = ros::Time().fromSec(m_sync_pack.scan_time);
    points.points.reserve(m_sync_pack.scan.size());
    for (Vec2f &p : m_sync_pack.scan)
    {
        geometry_msgs::Point32 point;
        point.x = p.x();
        point.y = p.y();
        point.z = 0;
        points.points.push_back(point);
    }
    m_scan_pub.publish(points);
}

void VoxelMappingROS::sendTransform()
{
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time().fromSec(m_sync_pack.scan_time);
    tf_msg.header.frame_id = m_config.map_frame;
    tf_msg.child_frame_id = m_config.body_frame;
    tf_msg.transform.translation.x = m_map_builder->isefk2d.X.pos.x();
    tf_msg.transform.translation.y = m_map_builder->isefk2d.X.pos.y();
    tf_msg.transform.rotation = tf::createQuaternionMsgFromYaw(m_map_builder->isefk2d.X.theta);
    m_tf_broadcaster.sendTransform(tf_msg);
}
void VoxelMappingROS::publishOdometry()
{
    if (m_odom_pub.getNumSubscribers() == 0)
        return;
    XState &state = m_map_builder->isefk2d.X;
    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time().fromSec(m_sync_pack.scan_time);
    msg.header.frame_id = m_config.map_frame;
    msg.child_frame_id = m_config.body_frame;
    msg.pose.pose.position.x = state.pos.x();
    msg.pose.pose.position.y = state.pos.y();
    msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(state.theta);
    Vec2f vel = Eigen::Rotation2Df(state.theta).inverse() * state.vel;
    msg.twist.twist.linear.x = vel.x();
    msg.twist.twist.linear.y = vel.y();
    msg.twist.twist.angular.z = (m_sync_pack.imus.back().gyro.z() - state.bg);
    m_odom_pub.publish(msg);
}