#include "cloud_generator.h"

void CloudGenerator::addCloud(const Vec3f &pose, const std::vector<Vec2f> &cloud)
{
    if (!m_poses.empty())
    {
        Vec3f last_pose = m_poses.back();
        if ((pose - last_pose).head<2>().norm() < m_config.translation_thresh && fabs(normalize_theta(pose.z() - last_pose.z())) < m_config.rotation_thresh)
            return;
    }
    Eigen::Affine2f trans = Eigen::Translation2f(pose.x(), pose.y()) * Eigen::Rotation2Df(pose.z());
    for (const auto &point : cloud)
    {
        Vec2f p = trans * point;
        if (p.x() > m_state.max_point.x())
            m_state.max_point.x() = p.x();
        if (p.y() > m_state.max_point.y())
            m_state.max_point.y() = p.y();
        if (p.x() < m_state.min_point.x())
            m_state.min_point.x() = p.x();
        if (p.y() < m_state.min_point.y())
            m_state.min_point.y() = p.y();
        m_cloud.push_back(p);
    }
    m_poses.push_back(pose);
    m_indices.push_back(m_cloud.size());
}

void CloudGenerator::saveCloud(const std::string &filename)
{
    if (m_cloud.empty())
        return;
    pcl::PCDWriter writer;
    pcl::PointCloud<pcl::PointXYZ> pcl_clouds;
    pcl_clouds.reserve(m_cloud.size());
    for (const auto &point : m_cloud)
        pcl_clouds.emplace_back(pcl::PointXYZ{point.x(), point.y(), 0.0});
    writer.writeBinaryCompressed(filename, pcl_clouds);
}

void CloudGenerator::saveGrid(const std::string &save_dir, const float &resolution, const float &occ_prob, const float &free_prob)
{
    int width = ceil((m_state.max_point.x() - m_state.min_point.x()) / resolution);
    int height = ceil((m_state.max_point.y() - m_state.min_point.y()) / resolution);

    std::vector<float> grid(width * height);
    std::fill(grid.begin(), grid.end(), 0.0);
    std::vector<int> visite_flag(width * height);
    std::fill(visite_flag.begin(), visite_flag.end(), 0);

    assert(occ_prob > 0.5f && occ_prob < 1.0f);
    assert(free_prob > 0.5f && free_prob < 1.0f);
    float occ_odds = log(occ_prob / (1 - occ_prob));
    float free_odds = log((1 - free_prob) / free_prob);
    unsigned int idx;
    int current_free_flag, current_occ_flag;

    for (int i = 0; i < m_poses.size(); ++i)
    {
        unsigned int s_idx = i == 0 ? 0 : m_indices[i - 1];
        unsigned int e_idx = m_indices[i];
        Vec2f pose_in_map = (m_poses[i].head<2>() - m_state.min_point) / resolution;
        current_free_flag = i * 2;
        current_occ_flag = i * 2 + 1;
        for (unsigned int j = s_idx; j < e_idx; ++j)
        {
            Vec2f point_in_map = (m_cloud[j] - m_state.min_point) / resolution;
            std::vector<std::pair<int, int>> line = draw_line(pose_in_map.x(), pose_in_map.y(), point_in_map.x(), point_in_map.y());
            // 更新free
            for (unsigned int k = 0; k < line.size() - 1; ++k)
            {
                idx = line[k].second * width + line[k].first;
                if (visite_flag[idx] < current_free_flag)
                {
                    visite_flag[idx] = current_free_flag;
                    grid[idx] += free_odds;
                    if (grid[idx] < -50.0f)
                        grid[idx] = -50.0f;
                }
            }
            if (line.size() == 1)
                continue;
            // 更新occ
            idx = line.back().second * width + line.back().first;
            if (visite_flag[idx] == current_free_flag)
            {
                grid[idx] -= free_odds;
                if (grid[idx] > 50.0f)
                    grid[idx] = 50.0f;
            }
            if (visite_flag[idx] < current_occ_flag)
            {
                visite_flag[idx] = current_occ_flag;
                grid[idx] += occ_odds;
                if (grid[idx] > 50.0f)
                    grid[idx] = 50.0f;
            }
        }
    }

    cv::Mat map(height, width, CV_8UC1);
    for (int i = 0; i < height; ++i)
    {
        for (int j = 0; j < width; ++j)
        {
            idx = (height - i - 1) * width + j;
            float exp_odd = exp(grid[idx]);
            float prob = exp_odd / (exp_odd + 1.0f);
            map.at<uchar>(i, j) = static_cast<uchar>(255.0f * (1 - prob));
        }
    }
    std::filesystem::path par_dir(save_dir);
    std::filesystem::path img_path = par_dir / "map.png";
    std::filesystem::path yaml_path = par_dir / "map.yaml";
    cv::imwrite(img_path.string(), map);

    // cv::FileStorage fs(yaml_path.string(), cv::FileStorage::WRITE);
    // fs << "image" << "map.png";
    // fs << "resolution" << resolution;
    // std::vector<float> origin{m_state.min_point.x(), m_state.min_point.y(), 0.0f};
    // fs << "origin" << origin;
    // fs << "negate" << true;
    // fs << "occupied_thresh" << 200;
    // fs << "free_thresh" << 100;
    // fs.release();
    YAML::Node node;
    node["image"] = "map.png";
    node["resolution"] = resolution;
    node["origin"] = YAML::Node(YAML::NodeType::Sequence);
    node["origin"][0] = m_state.min_point.x();
    node["origin"][1] = m_state.min_point.y();
    node["origin"][2] = 0.0f;
    node["negate"] = true;
    node["occupied_thresh"] = 200;
    node["free_thresh"] = 100;
    std::ofstream fout(yaml_path.string());
    fout << node;
    fout.close();
}