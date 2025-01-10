#include "voxel_mapping_ros.h"

VoxelMappingROS::VoxelMappingROS(ros::NodeHandle &nh) : m_nh(nh)
{
    loadParameters();
    initPublishers();
    initSubscribers();
    m_map_builder = std::make_shared<VoxelMapBuilder2D>(m_map_config);
}
void VoxelMappingROS::loadParameters()
{
    m_nh.param<std::string>("map_frame", m_config.map_frame, "map");
    m_nh.param<std::string>("scan_frame", m_config.scan_frame, "laser");
    m_nh.param<std::string>("scan_topic", m_config.scan_topic, "/scan");
    m_nh.param<float>("ignore_range", m_config.ignore_range, 0.1f);
    m_nh.param<float>("resolution", m_map_config.resolution, 0.05f);

    m_nh.param<float>("update_thresh", m_map_config.update_thresh, 5.0f);
    m_nh.param<float>("max_update_thresh", m_map_config.max_update_thresh, 100.0f);
    m_nh.param<int>("max_iteration", m_map_config.max_iteration, 10);
    m_nh.param<float>("plane_threshold", m_map_config.plane_threshold, 0.1f);
    m_nh.param<float>("rotate_thresh", m_map_config.rotate_thresh, 0.1f);
    m_nh.param<float>("move_thresh", m_map_config.move_thresh, 0.2f);
    m_nh.param<int>("max_voxel_number", m_map_config.max_voxel_number, 500000);
    m_nh.param<bool>("use_cluster", m_map_config.use_cluster, false);

    ROS_DEBUG("map_frame: %s", m_config.map_frame.c_str());
    ROS_DEBUG("scan_frame: %s", m_config.scan_frame.c_str());
    ROS_DEBUG("scan_topic: %s", m_config.scan_topic.c_str());
    ROS_DEBUG("ignore_range: %f", m_config.ignore_range);

    ROS_DEBUG("resolution: %f", m_map_config.resolution);
    ROS_DEBUG("update_thresh: %f", m_map_config.update_thresh);
    ROS_DEBUG("max_update_thresh: %f", m_map_config.max_update_thresh);
    ROS_DEBUG("max_iteration: %d", m_map_config.max_iteration);
    ROS_DEBUG("plane_threshold: %f", m_map_config.plane_threshold);
    ROS_DEBUG("rotate_thresh: %f", m_map_config.rotate_thresh);
    ROS_DEBUG("move_thresh: %f", m_map_config.move_thresh);
    ROS_DEBUG("use_cluster: %d", m_map_config.use_cluster);
    ROS_DEBUG("max_voxel_number: %d", m_map_config.max_voxel_number);
}
void VoxelMappingROS::initPublishers()
{
    m_scan_pub = m_nh.advertise<sensor_msgs::PointCloud>("points", 1);
}
void VoxelMappingROS::initSubscribers()
{
    m_scan_sub = m_nh.subscribe(m_config.scan_topic, 1, &VoxelMappingROS::scanCallback, this);
}
void VoxelMappingROS::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    setPackage(msg, m_package, m_config.ignore_range);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    m_map_builder->update(m_package);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    ROS_DEBUG("update time: %f ms", std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000);
    publishScanPoints();
    sendTransform();
}

void VoxelMappingROS::publishScanPoints()
{
    if (m_package.points.empty() || m_scan_pub.getNumSubscribers() == 0)
        return;
    sensor_msgs::PointCloud points;
    points.header.frame_id = m_config.scan_frame;
    points.header.stamp = ros::Time().fromSec(m_package.time);
    points.points.reserve(m_package.points.size());
    for (Vec2f &p : m_package.points)
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
    tf_msg.header.stamp = ros::Time().fromSec(m_package.time);
    tf_msg.header.frame_id = m_config.map_frame;
    tf_msg.child_frame_id = m_config.scan_frame;
    tf_msg.transform.translation.x = m_package.pose.x();
    tf_msg.transform.translation.y = m_package.pose.y();
    tf_msg.transform.rotation = tf::createQuaternionMsgFromYaw(m_package.pose.z());
    m_tf_broadcaster.sendTransform(tf_msg);
}