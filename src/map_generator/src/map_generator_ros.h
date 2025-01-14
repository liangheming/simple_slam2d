#pragma once
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include "generators/cloud_generator.h"
#include "interfaces/StringTrigger.h"
#include <filesystem>

struct NodeConfig
{
    std::string odom_topic;
    std::string cloud_topic;
};

class MapGeneratorROS
{
public:
    MapGeneratorROS(ros::NodeHandle &nh);
    void loadParameters();
    void initSubscribers();
    void initPublishers();
    void initServices();
    void syncCallback(const sensor_msgs::PointCloudConstPtr &cloud_msg, const nav_msgs::OdometryConstPtr &odom_msg);
    bool saveCloudSRVCB(interfaces::StringTrigger::Request &req, interfaces::StringTrigger::Response &res);

private:
    ros::NodeHandle m_nh;
    NodeConfig m_config;
    message_filters::Subscriber<sensor_msgs::PointCloud> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::Odometry> m_odom_sub;
    message_filters::Synchronizer<message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud, nav_msgs::Odometry>> m_sync;
    CloudGeneratorConfig m_cloud_generator_config;
    std::shared_ptr<CloudGenerator> m_cloud_generator;

    ros::ServiceServer m_save_cloud_srv;
};