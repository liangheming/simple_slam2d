#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/tf.h>

#include <queue>
#include <mutex>
#include "map_builder/voxel_map2d.h"
#include <thread>

struct NodeConfig
{
    std::string scan_topic;
    std::string imu_topic;
    std::string map_frame;
    std::string body_frame;
    float ignore_range{0.5f};
};
struct NodeState
{
    std::queue<sensor_msgs::Imu::ConstPtr> imu_queue;
    std::queue<sensor_msgs::LaserScan::ConstPtr> scan_queue;
    std::mutex imu_mutex;
    std::mutex scan_mutex;
    std::mutex system_mutex;
    double imu_last_time{0.0};
    double scan_last_time{0.0};
    bool scan_pushed{false};
    bool is_activated{true};
};
class VoxelMappingROS
{
public:
    VoxelMappingROS(ros::NodeHandle &nh);
    void loadParameters();
    void initPublishers();
    void initSubscribers();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void mainLoop();
    bool syncScanImu();
    void filterScan(const sensor_msgs::LaserScan::ConstPtr &msg, std::vector<Vec2f> &scan);
    void publishScanPoints();
    void sendTransform();
    void publishOdometry();

private:
    NodeConfig m_config;
    NodeState m_state;
    std::thread m_main_loop;
    Config m_map_config;
    SyncPack m_sync_pack;
    ros::NodeHandle m_nh;
    ros::Subscriber m_scan_sub;
    ros::Subscriber m_imu_sub;
    ros::Publisher m_scan_pub;
    ros::Publisher m_odom_pub;
    tf2_ros::TransformBroadcaster m_tf_broadcaster;
    std::shared_ptr<VoxelMapBuilder2D> m_map_builder;
};