#include "map_generator_ros.h"

MapGeneratorROS::MapGeneratorROS(ros::NodeHandle &nh) : m_nh(nh), m_sync(10)
{
    loadParameters();
    initPublishers();
    initSubscribers();
    initServices();
    m_cloud_generator = std::make_shared<CloudGenerator>(m_cloud_generator_config);
}
void MapGeneratorROS::loadParameters()
{
    m_nh.param<std::string>("cloud_topic", m_config.cloud_topic, "points");
    m_nh.param<std::string>("odom_topic", m_config.odom_topic, "odom");
    m_nh.param<float>("rotation_thresh", m_cloud_generator_config.rotation_thresh, 0.1f);
    m_nh.param<float>("translation_thresh", m_cloud_generator_config.translation_thresh, 0.1f);

    ROS_DEBUG("cloud_topic: %s", m_config.cloud_topic.c_str());
    ROS_DEBUG("odom_topic: %s", m_config.odom_topic.c_str());
    ROS_DEBUG("rotation_thresh: %f", m_cloud_generator_config.rotation_thresh);
    ROS_DEBUG("translation_thresh: %f", m_cloud_generator_config.translation_thresh);
}
void MapGeneratorROS::initSubscribers()
{
    m_cloud_sub.subscribe(m_nh, m_config.cloud_topic, 10);
    m_odom_sub.subscribe(m_nh, m_config.odom_topic, 10);
    m_sync.connectInput(m_cloud_sub, m_odom_sub);
    m_sync.registerCallback(boost::bind(&MapGeneratorROS::syncCallback, this, _1, _2));
}
void MapGeneratorROS::initPublishers()
{
}
void MapGeneratorROS::syncCallback(const sensor_msgs::PointCloudConstPtr &cloud_msg, const nav_msgs::OdometryConstPtr &odom_msg)
{
    Vec3f pose(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, tf::getYaw(odom_msg->pose.pose.orientation));
    std::vector<Vec2f> cloud;
    cloud.reserve(cloud_msg->points.size());
    for (const auto &point : cloud_msg->points)
    {
        cloud.emplace_back(point.x, point.y);
    }
    m_cloud_generator->addCloud(pose, cloud);
}

void MapGeneratorROS::initServices()
{
    m_save_cloud_srv = m_nh.advertiseService("save_cloud", &MapGeneratorROS::saveCloudSRVCB, this);
    m_save_grid_srv = m_nh.advertiseService("save_grid", &MapGeneratorROS::saveGridSRVCB, this);
}
bool MapGeneratorROS::saveCloudSRVCB(interfaces::StringTrigger::Request &req, interfaces::StringTrigger::Response &res)
{
    std::filesystem::path file_path(req.data);
    std::filesystem::path file_dir = file_path.parent_path();
    if (!std::filesystem::exists(file_dir))
    {
        res.success = false;
        res.message = "Directory does not exist: " + file_dir.string();
        return true;
    }
    m_cloud_generator->saveCloud(file_path.string());
    res.success = true;
    res.message = "Cloud saved to: " + file_path.string();
    return true;
}
bool MapGeneratorROS::saveGridSRVCB(interfaces::SaveGridMap::Request &req, interfaces::SaveGridMap::Response &res)
{
    std::filesystem::path save_dir = req.save_dir;
    if (!std::filesystem::exists(save_dir))
    {
        res.success = false;
        res.message = "Directory does not exist: " + save_dir.string();
        return true;
    }
    if (req.resolution <= 0.0f || req.occ_prob <= 0.5f || req.free_prob <= 0.5f)
    {
        res.success = false;
        res.message = "Invalid parameters";
        return true;
    }
    m_cloud_generator->saveGrid(save_dir.string(), req.resolution, req.occ_prob, req.free_prob);

    res.message = "Grid saved to: " + save_dir.string();
    res.success = true;
    return true;
}