#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include "map_builder/voxel_map2d.h"

inline void setPackage(const sensor_msgs::LaserScan::ConstPtr &msg, ScanPack &scan_pack, float ignore_range)
{
    scan_pack.points.clear();
    scan_pack.time = msg->header.stamp.toSec();
    float min_range = msg->range_min + 0.05;
    float max_range = msg->range_max - 0.05;
    for (int i = 0; i < msg->ranges.size(); i++)
    {
        float range = msg->ranges[i];
        if (range < min_range || range > max_range || std::isinf(range) || std::isnan(range) || range < ignore_range)
            continue;
        float angle = normalize_theta(msg->angle_min + i * msg->angle_increment);
        scan_pack.points.push_back(Vec2f(range * cos(angle), range * sin(angle)));
    }
}

struct NodeConfig
{
    std::string scan_topic;
    std::string map_frame;
    std::string scan_frame;
    float ignore_range{0.5f};
};
class VoxelMappingROS
{
public:
    VoxelMappingROS(ros::NodeHandle &nh);
    void loadParameters();
    void initPublishers();
    void initSubscribers();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void publishScanPoints();
    void sendTransform();

private:
    NodeConfig m_config;
    Config m_map_config;
    ros::NodeHandle m_nh;
    ros::Subscriber m_scan_sub;
    ros::Publisher m_scan_pub;
    ScanPack m_package;
    tf2_ros::TransformBroadcaster m_tf_broadcaster;
    std::shared_ptr<VoxelMapBuilder2D> m_map_builder;
};