#pragma once
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include "icps/plicp.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <filesystem>

struct NodeConfig
{
    std::string map_frame;
    std::string point_topic;
    std::string odom_topic;
    std::string map_pcd_path;
};
struct NodeState
{
};

class LocalizerROS
{
public:
    LocalizerROS(ros::NodeHandle &nh);
    void loadParameters();
    void initSubscribers();
    void initPublishers();
    void initServices();
    void syncCallBack(const sensor_msgs::PointCloud::ConstPtr &cloud, const nav_msgs::Odometry::ConstPtr &odom);
    bool readPCD(std::string &pcd_path, std::vector<Vec2f> &points);
    void publishMapCloud(const double& timestamp);

private:
    ros::NodeHandle m_nh;
    NodeState m_state;
    NodeConfig m_config;
    PLICPConfig m_plicp_config;
    message_filters::Subscriber<nav_msgs::Odometry> m_odom_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud> m_point_sub;
    message_filters::Synchronizer<message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud, nav_msgs::Odometry>> m_sync;
    std::shared_ptr<PLICP> m_plicp;

    ros::Publisher m_map_pub;
};