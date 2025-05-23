#include "obstacle_detector.h"

ObstacleDetector::ObstacleDetector() 
    : nh_("~"),
      powerline_sub_(nh_, "", 1),
      downsampled_sub_(nh_, "", 1),
      powerline_cloud_(new pcl::PointCloud<pcl::PointXYZI>()),
      downsampled_cloud_(new pcl::PointCloud<pcl::PointXYZI>()),
      obstacle_cloud_(new pcl::PointCloud<pcl::PointXYZI>()),
      powerline_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZI>()),
      global_min_distance_(std::numeric_limits<float>::max()),
      powerline_received_(false),
      downsampled_received_(false),
      frame_count_(0) {
    
    // 读取参数
    nh_.param<std::string>("target_frame", target_frame_, "map");
    nh_.param<std::string>("powerline_topic", powerline_topic_, "/powerline_extractor/clustered_powerline_cloud");
    nh_.param<std::string>("downsampled_topic", downsampled_topic_, "/powerline_extractor/downsamole_cloud");
    
    // 障碍物检测参数
    nh_.param<float>("search_radius", search_radius_, 5.0);
    nh_.param<float>("min_obstacle_distance", min_obstacle_distance_, 2.0);
    nh_.param<float>("cluster_tolerance", cluster_tolerance_, 0.5);
    nh_.param<int>("min_cluster_size", min_cluster_size_, 10);
    nh_.param<int>("max_cluster_size", max_cluster_size_, 1000);
    nh_.param<float>("min_obstacle_height", min_obstacle_height_, 0.3);
    nh_.param<float>("max_obstacle_height", max_obstacle_height_, 10.0);
    
    // 可视化参数
    nh_.param<bool>("enable_visualization", enable_visualization_, true);
    nh_.param<bool>("show_distance_lines", show_distance_lines_, true);
    nh_.param<bool>("show_text_labels", show_text_labels_, true);
    nh_.param<float>("marker_lifetime", marker_lifetime_, 0.5);

    // 新增电力线排除参数
    nh_.param<float>("powerline_exclusion_radius", powerline_exclusion_radius_, 1.5);
    nh_.param<float>("min_obstacle_separation", min_obstacle_separation_, 2.0);
    nh_.param<bool>("use_intensity_filter", use_intensity_filter_, false);
    nh_.param<float>("powerline_intensity_threshold", powerline_intensity_threshold_, 100.0);
    
    checkParameters();
    
    // 设置消息同步器
    powerline_sub_.subscribe(nh_, powerline_topic_, 1);
    downsampled_sub_.subscribe(nh_, downsampled_topic_, 1);
    
    sync_.reset(new Synchronizer(SyncPolicy(10), powerline_sub_, downsampled_sub_));
    sync_->registerCallback(boost::bind(&ObstacleDetector::syncCallback, this, _1, _2));
    
    // 初始化发布器
    obstacle_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("obstacle_cloud", 1);
    markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacle_markers", 1);
    min_distance_pub_ = nh_.advertise<std_msgs::Float32>("min_distance", 1);
    
    ROS_INFO("Obstacle Detector initialized");
    ROS_INFO("Subscribing to: %s and %s", powerline_topic_.c_str(), downsampled_topic_.c_str());
}

ObstacleDetector::~ObstacleDetector() {
}

void ObstacleDetector::checkParameters() {
    ROS_INFO("=== Obstacle Detector Parameters ===");
    ROS_INFO("Target frame: %s", target_frame_.c_str());
    ROS_INFO("Powerline topic: %s", powerline_topic_.c_str());
    ROS_INFO("Downsampled topic: %s", downsampled_topic_.c_str());
    ROS_INFO("Search radius: %.2f m", search_radius_);
    ROS_INFO("Min obstacle distance: %.2f m", min_obstacle_distance_);
    ROS_INFO("Cluster tolerance: %.2f m", cluster_tolerance_);
    ROS_INFO("Min/Max cluster size: %d/%d", min_cluster_size_, max_cluster_size_);
    ROS_INFO("Obstacle height range: [%.2f, %.2f] m", min_obstacle_height_, max_obstacle_height_);
    
    // 新增参数显示
    ROS_INFO("Powerline exclusion radius: %.2f m", powerline_exclusion_radius_);
    ROS_INFO("Min obstacle separation: %.2f m", min_obstacle_separation_);
    ROS_INFO("Use intensity filter: %s", use_intensity_filter_ ? "Yes" : "No");
    if (use_intensity_filter_) {
        ROS_INFO("Powerline intensity threshold: %.1f", powerline_intensity_threshold_);
    }
    
    ROS_INFO("Visualization enabled: %s", enable_visualization_ ? "Yes" : "No");
    
    // 参数验证
    if (search_radius_ <= 0) {
        ROS_WARN("Invalid search_radius, using default 5.0");
        search_radius_ = 5.0;
    }
    
    if (min_obstacle_distance_ <= 0) {
        ROS_WARN("Invalid min_obstacle_distance, using default 2.0");
        min_obstacle_distance_ = 2.0;
    }
    
    if (cluster_tolerance_ <= 0) {
        ROS_WARN("Invalid cluster_tolerance, using default 0.5");
        cluster_tolerance_ = 0.5;
    }

    // 参数验证和调整
    if (powerline_exclusion_radius_ <= 0) {
        ROS_WARN("Invalid powerline_exclusion_radius, using default 1.5");
        powerline_exclusion_radius_ = 1.5;
    }
    
    if (min_obstacle_separation_ <= 0) {
        ROS_WARN("Invalid min_obstacle_separation, using default 2.0");
        min_obstacle_separation_ = 2.0;
    }
    
    // 确保排除半径小于搜索半径
    if (powerline_exclusion_radius_ >= search_radius_) {
        ROS_WARN("Powerline exclusion radius should be smaller than search radius");
        powerline_exclusion_radius_ = search_radius_ * 0.3;
        ROS_INFO("Adjusted powerline exclusion radius to %.2f m", powerline_exclusion_radius_);
    }

}

void ObstacleDetector::syncCallback(const sensor_msgs::PointCloud2ConstPtr& powerline_msg,
                                   const sensor_msgs::PointCloud2ConstPtr& downsampled_msg) {
    
    ROS_INFO_THROTTLE(5.0, "Received synchronized point clouds: powerline(%d points), downsampled(%d points)",
                     powerline_msg->width * powerline_msg->height,
                     downsampled_msg->width * downsampled_msg->height);
    
    // 转换点云消息到PCL格式
    pcl::fromROSMsg(*powerline_msg, *powerline_cloud_);
    pcl::fromROSMsg(*downsampled_msg, *downsampled_cloud_);
    
    powerline_received_ = true;
    downsampled_received_ = true;
    
    // 检查点云是否为空
    if (powerline_cloud_->empty()) {
        ROS_WARN("Received empty powerline cloud");
        return;
    }
    
    if (downsampled_cloud_->empty()) {
        ROS_WARN("Received empty downsampled cloud");
        return;
    }
    
    // 处理障碍物检测
    detectObstacles();
    
    frame_count_++;
    last_process_time_ = ros::Time::now();
}

void ObstacleDetector::detectObstacles() {
    ros::WallTime start_time = ros::WallTime::now();
    
    // 清空之前的结果
    detected_obstacles_.clear();
    obstacle_cloud_->clear();
    global_min_distance_ = std::numeric_limits<float>::max();
    
    // 1. 提取电力线周围的障碍物候选点
    extractObstaclesAroundPowerlines();
    
    if (obstacle_cloud_->empty()) {
        ROS_INFO("No obstacle candidates found");
        publishVisualization();
        return;
    }
    
    // 2. 聚类障碍物
    clusterObstacles(obstacle_cloud_);
    
    // 3. 计算距离
    calculateDistancesToPowerlines();
    
    // 4. 发布结果
    publishVisualization();
    publishMinDistance();
    
    // 性能统计
    ros::WallTime end_time = ros::WallTime::now();
    double execution_time = (end_time - start_time).toSec() * 1000.0;
    
    ROS_INFO("Obstacle detection completed in %.2f ms: found %zu obstacles, min distance: %.2f m",
             execution_time, detected_obstacles_.size(), global_min_distance_);
}

// void ObstacleDetector::extractObstaclesAroundPowerlines() {
//     if (powerline_cloud_->empty() || downsampled_cloud_->empty()) {
//         return;
//     }
    
//     // 建立电力线点云的KD树
//     powerline_kdtree_->setInputCloud(powerline_cloud_);
    
//     std::vector<int> pointIdxRadiusSearch;
//     std::vector<float> pointRadiusSquaredDistance;
    
//     // 遍历下采样点云，寻找电力线周围的点
//     for (const auto& point : downsampled_cloud_->points) {
//         // 检查点的有效性
//         if (!pcl::isFinite(point)) {
//             continue;
//         }
        
//         // 检查高度范围
//         if (point.z < min_obstacle_height_ || point.z > max_obstacle_height_) {
//             continue;
//         }
        
//         // 搜索最近的电力线点
//         if (powerline_kdtree_->radiusSearch(point, search_radius_, 
//                                           pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
            
//             // 计算到最近电力线点的距离
//             float min_dist = std::sqrt(pointRadiusSquaredDistance[0]);
            
//             // 如果距离在指定范围内，但不是电力线点本身
//             if (min_dist > 0.1 && min_dist < search_radius_) {  // 0.1m阈值避免包含电力线点本身
                
//                 // 检查是否真的是障碍物（不是电力线点）
//                 if (!isPointNearPowerline(point, 0.5)) {  // 0.5m内认为是电力线
//                     obstacle_cloud_->push_back(point);
//                 }
//             }
//         }
//     }
    
//     ROS_INFO("Extracted %zu obstacle candidate points around powerlines", obstacle_cloud_->size());
// }


// 完全重写extractObstaclesAroundPowerlines函数
void ObstacleDetector::extractObstaclesAroundPowerlines() {
    if (powerline_cloud_->empty() || downsampled_cloud_->empty()) {
        return;
    }
    
    // 建立电力线点云的KD树
    powerline_kdtree_->setInputCloud(powerline_cloud_);
    
    // 第一步：创建电力线排除区域
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    
    int total_points = 0;
    int excluded_by_powerline = 0;
    int excluded_by_height = 0;
    int excluded_by_intensity = 0;
    int valid_candidates = 0;
    
    for (const auto& point : downsampled_cloud_->points) {
        total_points++;
        
        // 检查点的有效性
        if (!pcl::isFinite(point)) {
            continue;
        }
        
        // 检查高度范围
        if (point.z < min_obstacle_height_ || point.z > max_obstacle_height_) {
            excluded_by_height++;
            continue;
        }
        
        // 强度过滤（可选）- 如果电力线有特定的强度特征
        if (use_intensity_filter_ && point.intensity > powerline_intensity_threshold_) {
            excluded_by_intensity++;
            continue;
        }
        
        // 检查是否在电力线排除区域内
        if (isPointInPowerlineExclusionZone(point, powerline_exclusion_radius_)) {
            excluded_by_powerline++;
            continue;
        }
        
        // 检查是否在电力线搜索半径内（潜在障碍物区域）
        if (isPointInPowerlineSearchRadius(point, search_radius_)) {
            filtered_cloud->push_back(point);
            valid_candidates++;
        }
    }
    
    ROS_INFO("Point filtering results:");
    ROS_INFO("  Total points: %d", total_points);
    ROS_INFO("  Excluded by height: %d", excluded_by_height);
    ROS_INFO("  Excluded by powerline zone: %d", excluded_by_powerline);
    if (use_intensity_filter_) {
        ROS_INFO("  Excluded by intensity: %d", excluded_by_intensity);
    }
    ROS_INFO("  Valid obstacle candidates: %d", valid_candidates);
    
    // 第二步：进一步过滤，确保障碍物与电力线有足够分离
    for (const auto& point : filtered_cloud->points) {
        float min_distance_to_powerline = calculateMinDistanceToPoint(point, powerline_cloud_);
        
        // 只有距离电力线足够远的点才被认为是障碍物
        if (min_distance_to_powerline >= min_obstacle_separation_) {
            obstacle_cloud_->push_back(point);
        }
    }
    
    ROS_INFO("Final obstacle candidates after separation filter: %zu points", obstacle_cloud_->size());
}

// 新增辅助函数：检查点是否在电力线排除区域内
bool ObstacleDetector::isPointInPowerlineExclusionZone(const pcl::PointXYZI& point, float exclusion_radius) {
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    
    // 在排除半径内搜索电力线点
    if (powerline_kdtree_->radiusSearch(point, exclusion_radius, 
                                      pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        return true;  // 在排除区域内
    }
    
    return false;  // 不在排除区域内
}

// 新增辅助函数：检查点是否在电力线搜索半径内
bool ObstacleDetector::isPointInPowerlineSearchRadius(const pcl::PointXYZI& point, float search_radius) {
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    
    // 在搜索半径内查找电力线点
    if (powerline_kdtree_->radiusSearch(point, search_radius, 
                                      pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        return true;  // 在搜索区域内
    }
    
    return false;  // 不在搜索区域内
}


// bool ObstacleDetector::isPointNearPowerline(const pcl::PointXYZI& point, float max_distance) {
//     std::vector<int> pointIdxNearestSearch(1);
//     std::vector<float> pointNearestSquaredDistance(1);
    
//     if (powerline_kdtree_->nearestKSearch(point, 1, pointIdxNearestSearch, pointNearestSquaredDistance) > 0) {
//         return std::sqrt(pointNearestSquaredDistance[0]) < max_distance;
//     }
    
//     return false;
// }

// 改进的isPointNearPowerline函数（保留原有功能但优化性能）
bool ObstacleDetector::isPointNearPowerline(const pcl::PointXYZI& point, float max_distance) {
    std::vector<int> pointIdxNearestSearch(1);
    std::vector<float> pointNearestSquaredDistance(1);
    
    if (powerline_kdtree_->nearestKSearch(point, 1, pointIdxNearestSearch, pointNearestSquaredDistance) > 0) {
        float distance = std::sqrt(pointNearestSquaredDistance[0]);
        return distance < max_distance;
    }
    
    return false;
}

// void ObstacleDetector::clusterObstacles(const pcl::PointCloud<pcl::PointXYZI>::Ptr& obstacle_candidates) {
//     if (obstacle_candidates->empty()) {
//         return;
//     }
    
//     // 创建聚类对象
//     pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
//     tree->setInputCloud(obstacle_candidates);
    
//     std::vector<pcl::PointIndices> cluster_indices;
//     pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
//     ec.setClusterTolerance(cluster_tolerance_);
//     ec.setMinClusterSize(min_cluster_size_);
//     ec.setMaxClusterSize(max_cluster_size_);
//     ec.setSearchMethod(tree);
//     ec.setInputCloud(obstacle_candidates);
//     ec.extract(cluster_indices);
    
//     ROS_INFO("Found %zu obstacle clusters", cluster_indices.size());
    
//     // 处理每个聚类
//     for (size_t i = 0; i < cluster_indices.size(); ++i) {
//         ObstacleInfo obstacle;
//         obstacle.obstacle_id = static_cast<int>(i);
        
//         // 提取聚类点
//         for (const auto& index : cluster_indices[i].indices) {
//             obstacle.obstacle_cloud->push_back(obstacle_candidates->points[index]);
//         }
        
//         // 计算边界框
//         pcl::PointXYZI min_pt, max_pt;
//         pcl::getMinMax3D(*obstacle.obstacle_cloud, min_pt, max_pt);
        
//         obstacle.min_point = Eigen::Vector3f(min_pt.x, min_pt.y, min_pt.z);
//         obstacle.max_point = Eigen::Vector3f(max_pt.x, max_pt.y, max_pt.z);
//         obstacle.center_point = (obstacle.min_point + obstacle.max_point) / 2.0f;
        
//         detected_obstacles_.push_back(obstacle);
//     }
// }



// 改进的clusterObstacles函数，添加额外的电力线点过滤
void ObstacleDetector::clusterObstacles(const pcl::PointCloud<pcl::PointXYZI>::Ptr& obstacle_candidates) {
    if (obstacle_candidates->empty()) {
        return;
    }
    
    // 创建聚类对象
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(obstacle_candidates);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(obstacle_candidates);
    ec.extract(cluster_indices);
    
    ROS_INFO("Found %zu preliminary obstacle clusters", cluster_indices.size());
    
    // 处理每个聚类并进行最终验证
    int valid_obstacles = 0;
    for (size_t i = 0; i < cluster_indices.size(); ++i) {
        ObstacleInfo obstacle;
        obstacle.obstacle_id = static_cast<int>(valid_obstacles);
        
        // 提取聚类点
        for (const auto& index : cluster_indices[i].indices) {
            obstacle.obstacle_cloud->push_back(obstacle_candidates->points[index]);
        }
        
        // 计算边界框
        pcl::PointXYZI min_pt, max_pt;
        pcl::getMinMax3D(*obstacle.obstacle_cloud, min_pt, max_pt);
        
        obstacle.min_point = Eigen::Vector3f(min_pt.x, min_pt.y, min_pt.z);
        obstacle.max_point = Eigen::Vector3f(max_pt.x, max_pt.y, max_pt.z);
        obstacle.center_point = (obstacle.min_point + obstacle.max_point) / 2.0f;
        
        // 验证聚类中心是否远离电力线
        pcl::PointXYZI center_point_pcl;
        center_point_pcl.x = obstacle.center_point.x();
        center_point_pcl.y = obstacle.center_point.y();
        center_point_pcl.z = obstacle.center_point.z();
        
        float center_distance_to_powerline = calculateMinDistanceToPoint(center_point_pcl, powerline_cloud_);
        
        // 只有中心距离电力线足够远的聚类才被认为是有效障碍物
        if (center_distance_to_powerline >= min_obstacle_separation_) {
            detected_obstacles_.push_back(obstacle);
            valid_obstacles++;
            
            ROS_DEBUG("Valid obstacle %d: center distance to powerline = %.2f m", 
                     valid_obstacles, center_distance_to_powerline);
        } else {
            ROS_DEBUG("Rejected cluster %zu: center too close to powerline (%.2f m < %.2f m)", 
                     i, center_distance_to_powerline, min_obstacle_separation_);
        }
    }
    
    ROS_INFO("Valid obstacles after final filtering: %d", valid_obstacles);
}


void ObstacleDetector::calculateDistancesToPowerlines() {
    if (powerline_cloud_->empty() || detected_obstacles_.empty()) {
        return;
    }
    
    for (auto& obstacle : detected_obstacles_) {
        float min_distance = std::numeric_limits<float>::max();
        
        // 计算障碍物中每个点到电力线的最小距离
        for (const auto& obs_point : obstacle.obstacle_cloud->points) {
            float distance = calculateMinDistanceToPoint(obs_point, powerline_cloud_);
            if (distance < min_distance) {
                min_distance = distance;
            }
        }
        
        obstacle.min_distance_to_powerline = min_distance;
        
        // 更新全局最小距离
        if (min_distance < global_min_distance_) {
            global_min_distance_ = min_distance;
            
            // 找到最近的点对
            for (const auto& obs_point : obstacle.obstacle_cloud->points) {
                float distance = calculateMinDistanceToPoint(obs_point, powerline_cloud_);
                if (std::abs(distance - min_distance) < 0.01) {  // 找到最近的点
                    closest_obstacle_point_ = Eigen::Vector3f(obs_point.x, obs_point.y, obs_point.z);
                    
                    // 找到对应的电力线点
                    std::vector<int> pointIdxNearestSearch(1);
                    std::vector<float> pointNearestSquaredDistance(1);
                    
                    if (powerline_kdtree_->nearestKSearch(obs_point, 1, 
                                                        pointIdxNearestSearch, pointNearestSquaredDistance) > 0) {
                        const auto& pl_point = powerline_cloud_->points[pointIdxNearestSearch[0]];
                        closest_powerline_point_ = Eigen::Vector3f(pl_point.x, pl_point.y, pl_point.z);
                    }
                    break;
                }
            }
        }
    }
    
    ROS_INFO("Calculated distances for %zu obstacles, global minimum: %.2f m", 
             detected_obstacles_.size(), global_min_distance_);
}

float ObstacleDetector::calculateMinDistanceToPoint(const pcl::PointXYZI& point, 
                                                  const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud) {
    std::vector<int> pointIdxNearestSearch(1);
    std::vector<float> pointNearestSquaredDistance(1);
    
    if (powerline_kdtree_->nearestKSearch(point, 1, pointIdxNearestSearch, pointNearestSquaredDistance) > 0) {
        return std::sqrt(pointNearestSquaredDistance[0]);
    }
    
    return std::numeric_limits<float>::max();
}

void ObstacleDetector::publishVisualization() {
    if (!enable_visualization_) {
        return;
    }
    
    marker_array_.markers.clear();
    int marker_id = 0;
    
    // 创建电力线边界框
    createPowerlineBoundingBox();
    
    // 为每个障碍物创建边界框
    for (const auto& obstacle : detected_obstacles_) {
        visualization_msgs::Marker bbox_marker;
        createBoundingBoxMarker(obstacle, bbox_marker);
        bbox_marker.id = marker_id++;
        marker_array_.markers.push_back(bbox_marker);
        
        // 创建距离线标记
        if (show_distance_lines_ && global_min_distance_ < std::numeric_limits<float>::max()) {
            createDistanceLineMarker(closest_powerline_point_, closest_obstacle_point_, 
                                   global_min_distance_, marker_id++);
        }
        
        // 创建文本标记
        if (show_text_labels_) {
            std::string text = "Obstacle " + std::to_string(obstacle.obstacle_id) + 
                             "\nDist: " + std::to_string(obstacle.min_distance_to_powerline).substr(0, 4) + "m";
            createTextMarker(obstacle.center_point, text, marker_id++);
        }
    }
    
    // 发布标记
    markers_pub_.publish(marker_array_);
    
    // 发布障碍物点云
    if (!obstacle_cloud_->empty()) {
        sensor_msgs::PointCloud2 obstacle_msg;
        pcl::toROSMsg(*obstacle_cloud_, obstacle_msg);
        obstacle_msg.header.frame_id = target_frame_;
        obstacle_msg.header.stamp = ros::Time::now();
        obstacle_cloud_pub_.publish(obstacle_msg);
    }
}

void ObstacleDetector::createPowerlineBoundingBox() {
    if (powerline_cloud_->empty()) {
        return;
    }
    
    // 计算电力线边界框
    pcl::PointXYZI min_pt, max_pt;
    pcl::getMinMax3D(*powerline_cloud_, min_pt, max_pt);
    
    visualization_msgs::Marker powerline_bbox;
    powerline_bbox.header.frame_id = target_frame_;
    powerline_bbox.header.stamp = ros::Time::now();
    powerline_bbox.ns = "powerline_bbox";
    powerline_bbox.id = 1000;  // 特殊ID用于电力线
    powerline_bbox.type = visualization_msgs::Marker::CUBE;
    powerline_bbox.action = visualization_msgs::Marker::ADD;
    
    // 设置位置和尺寸
    powerline_bbox.pose.position.x = (min_pt.x + max_pt.x) / 2.0;
    powerline_bbox.pose.position.y = (min_pt.y + max_pt.y) / 2.0;
    powerline_bbox.pose.position.z = (min_pt.z + max_pt.z) / 2.0;
    powerline_bbox.pose.orientation.w = 1.0;
    
    powerline_bbox.scale.x = max_pt.x - min_pt.x + 0.1;
    powerline_bbox.scale.y = max_pt.y - min_pt.y + 0.1;
    powerline_bbox.scale.z = max_pt.z - min_pt.z + 0.1;
    
    // 设置颜色（蓝色，半透明）
    powerline_bbox.color.r = 0.0;
    powerline_bbox.color.g = 0.0;
    powerline_bbox.color.b = 1.0;
    powerline_bbox.color.a = 0.3;
    
    powerline_bbox.lifetime = ros::Duration(marker_lifetime_);
    
    marker_array_.markers.push_back(powerline_bbox);
}

void ObstacleDetector::createBoundingBoxMarker(const ObstacleInfo& obstacle, 
                                              visualization_msgs::Marker& marker) {
    marker.header.frame_id = target_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "obstacle_bbox";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 设置位置和尺寸
    marker.pose.position.x = obstacle.center_point.x();
    marker.pose.position.y = obstacle.center_point.y();
    marker.pose.position.z = obstacle.center_point.z();
    marker.pose.orientation.w = 1.0;
    
    Eigen::Vector3f size = obstacle.max_point - obstacle.min_point;
    marker.scale.x = size.x() + 0.1;
    marker.scale.y = size.y() + 0.1;
    marker.scale.z = size.z() + 0.1;
    
    // 根据距离设置颜色（红色表示近，绿色表示远）
    float normalized_distance = std::min(obstacle.min_distance_to_powerline / min_obstacle_distance_, 1.0f);
    marker.color.r = 1.0 - normalized_distance;
    marker.color.g = normalized_distance;
    marker.color.b = 0.0;
    marker.color.a = 0.6;
    
    marker.lifetime = ros::Duration(marker_lifetime_);
}

void ObstacleDetector::createDistanceLineMarker(const Eigen::Vector3f& point1, 
                                               const Eigen::Vector3f& point2, 
                                               float distance, int id) {
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = target_frame_;
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "distance_line";
    line_marker.id = id;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;
    
    line_marker.scale.x = 0.05;  // 线宽
    
    // 设置颜色（黄色）
    line_marker.color.r = 1.0;
    line_marker.color.g = 1.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;
    
    // 添加两个点
    geometry_msgs::Point p1, p2;
    p1.x = point1.x(); p1.y = point1.y(); p1.z = point1.z();
    p2.x = point2.x(); p2.y = point2.y(); p2.z = point2.z();
    
    line_marker.points.push_back(p1);
    line_marker.points.push_back(p2);
    
    line_marker.lifetime = ros::Duration(marker_lifetime_);
    
    marker_array_.markers.push_back(line_marker);
    
    // 在线的中点添加距离文本
    Eigen::Vector3f mid_point = (point1 + point2) / 2.0f;
    std::string distance_text = "MIN: " + std::to_string(distance).substr(0, 4) + "m";
    createTextMarker(mid_point, distance_text, id + 1000);
}

void ObstacleDetector::createTextMarker(const Eigen::Vector3f& position, 
                                       const std::string& text, int id) {
    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = target_frame_;
    text_marker.header.stamp = ros::Time::now();
    text_marker.ns = "text_labels";
    text_marker.id = id;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    
    text_marker.pose.position.x = position.x();
    text_marker.pose.position.y = position.y();
    text_marker.pose.position.z = position.z() + 0.5;  // 稍微上移
    text_marker.pose.orientation.w = 1.0;
    
    text_marker.scale.z = 0.3;  // 文字大小
    
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    
    text_marker.text = text;
    text_marker.lifetime = ros::Duration(marker_lifetime_);
    
    marker_array_.markers.push_back(text_marker);
}

void ObstacleDetector::publishMinDistance() {
    std_msgs::Float32 min_dist_msg;
    min_dist_msg.data = global_min_distance_;
    min_distance_pub_.publish(min_dist_msg);
}