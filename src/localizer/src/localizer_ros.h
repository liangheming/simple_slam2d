#pragma once
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include "icps/plicp.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <mutex>
#include <thread>
#include <queue>

#include <filesystem>
#include <tf2_ros/transform_broadcaster.h>
#include "interfaces/ReLocalize.h"

struct NodeConfig
{
    std::string map_frame;
    std::string odom_frame;
    std::string body_frame;
    std::string point_topic;
    std::string odom_topic;
    std::string map_pcd_path;
    float localize_hz;
    int localize_duration;
    int max_acc_num{5};
    float scan_resolution{0.05};
};
struct NodeState
{
    std::queue<std::pair<nav_msgs::Odometry::ConstPtr, sensor_msgs::PointCloud::ConstPtr>> queue;
    Vec3f pose_diff{Vec3f::Zero()};
    Vec3f reloc_pose{Vec3f::Zero()};
    bool relocalize_called{false};
    std::mutex mutex;
    std::mutex reloc_mutex;
    double last_time{-1.0};
    double send_time{-1.0};
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
    void publishMapCloud(const double &timestamp);
    void publishMergedCloud(const std::vector<Vec2f> &points,const double &timestamp);
    void mainLoop();
    void sendtf(const ros::TimerEvent &);

    void initPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose);
    bool relocalizeCallBack(interfaces::ReLocalize::Request &req, interfaces::ReLocalize::Response &res);

private:
    ros::NodeHandle m_nh;
    NodeState m_state;
    NodeConfig m_config;
    PLICPConfig m_plicp_config;
    message_filters::Subscriber<nav_msgs::Odometry> m_odom_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud> m_point_sub;
    message_filters::Synchronizer<message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud, nav_msgs::Odometry>> m_sync;
    std::shared_ptr<PLICP> m_plicp;

    ros::Subscriber m_init_pose_sub;
    ros::Publisher m_map_pub;
    std::thread m_thread;
    ros::Timer m_tf_timer;
    tf2_ros::TransformBroadcaster m_tf_broadcaster;
    ros::Publisher m_point_pub;
    ros::ServiceServer m_relocalize_srv;
};