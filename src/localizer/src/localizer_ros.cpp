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
    ROS_INFO("LocalizerROS::LocalizerROS cloud_size: %lu", m_plicp->targetCloud().size());
    publishMapCloud(ros::Time::now().toSec());
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

    m_nh.param<int>("k_near", m_plicp_config.k_near, 10);
    m_nh.param<float>("resolution", m_plicp_config.resolution, 0.1);
    m_nh.param<int>("max_iteration", m_plicp_config.max_iteration, 10);

    ROS_DEBUG("LocalizerROS::loadParameters");
    ROS_DEBUG("k_near: %d", m_plicp_config.k_near);
    ROS_DEBUG("resolution: %f", m_plicp_config.resolution);
    ROS_DEBUG("max_iteration: %d", m_plicp_config.max_iteration);

    ROS_DEBUG("odom_topic: %s", m_config.odom_topic.c_str());
    ROS_DEBUG("point_topic: %s", m_config.point_topic.c_str());
    ROS_DEBUG("map_pcd_path: %s", m_config.map_pcd_path.c_str());
    ROS_DEBUG("map_frame: %s", m_config.map_frame.c_str());
}
void LocalizerROS::initSubscribers()
{
    m_odom_sub.subscribe(m_nh, m_config.odom_topic, 1);
    m_point_sub.subscribe(m_nh, m_config.point_topic, 1);
    m_sync.connectInput(m_point_sub, m_odom_sub);
    m_sync.registerCallback(boost::bind(&LocalizerROS::syncCallBack, this, _1, _2));
}
void LocalizerROS::initPublishers()
{
    m_map_pub = m_nh.advertise<sensor_msgs::PointCloud>("map", 1, true);
}
void LocalizerROS::initServices()
{
}
void LocalizerROS::syncCallBack(const sensor_msgs::PointCloud::ConstPtr &cloud, const nav_msgs::Odometry::ConstPtr &odom)
{
    float score;
    Vec3f pose = Vec3f(odom->pose.pose.position.x, odom->pose.pose.position.y, tf::getYaw(odom->pose.pose.orientation));
    std::vector<Vec2f> points;
    points.reserve(cloud->points.size());
    for (auto &point : cloud->points)
        points.emplace_back(point.x, point.y);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    pose.x() = 0.5;
    pose.y() = 0.5;
    pose.z() = 0.17;
    m_plicp->align(points, score, pose);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000.0;
    ROS_INFO("LocalizerROS::syncCallBack: score: %f, pose: %f, %f, %f, time: %f ms", score, pose.x(), pose.y(), pose.z(), ms);
    exit(0);
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