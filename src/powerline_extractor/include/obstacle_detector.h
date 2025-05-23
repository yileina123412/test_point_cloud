#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Dense>
#include <vector>
#include <string>



// 障碍物信息结构体
struct ObstacleInfo {
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud;
    Eigen::Vector3f min_point;
    Eigen::Vector3f max_point;
    Eigen::Vector3f center_point;
    float min_distance_to_powerline;
    int obstacle_id;
    
    ObstacleInfo() : obstacle_cloud(new pcl::PointCloud<pcl::PointXYZI>()) {}
};

class ObstacleDetector {
public:
    ObstacleDetector();
    ~ObstacleDetector();

private:
    // 同步回调函数
    void syncCallback(const sensor_msgs::PointCloud2ConstPtr& powerline_msg,
                     const sensor_msgs::PointCloud2ConstPtr& downsampled_msg);
    
    // 主要处理函数
    void detectObstacles();
    void calculateDistancesToPowerlines();
    void publishVisualization();
    void publishMinDistance();
    
    // 障碍物检测相关
    void extractObstaclesAroundPowerlines();
    void clusterObstacles(const pcl::PointCloud<pcl::PointXYZI>::Ptr& obstacle_candidates);
    float calculateMinDistanceToPoint(const pcl::PointXYZI& point, 
                                    const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud);
    
    // 新增的电力线过滤函数
    bool isPointInPowerlineExclusionZone(const pcl::PointXYZI& point, float exclusion_radius);
    bool isPointInPowerlineSearchRadius(const pcl::PointXYZI& point, float search_radius);
    
    // 可视化相关
    void createBoundingBoxMarker(const ObstacleInfo& obstacle, 
                               visualization_msgs::Marker& marker);
    void createPowerlineBoundingBox();
    void createDistanceLineMarker(const Eigen::Vector3f& point1, 
                                const Eigen::Vector3f& point2, 
                                float distance, int id);
    void createTextMarker(const Eigen::Vector3f& position, 
                        const std::string& text, int id);
    
    // 工具函数
    void checkParameters();
    bool isPointNearPowerline(const pcl::PointXYZI& point, float max_distance);
    
    // ROS相关
    ros::NodeHandle nh_;
    
    // 消息同步器
    message_filters::Subscriber<sensor_msgs::PointCloud2> powerline_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> downsampled_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, 
                                                           sensor_msgs::PointCloud2> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    boost::shared_ptr<Synchronizer> sync_;
    
    // 发布器
    ros::Publisher obstacle_cloud_pub_;
    ros::Publisher markers_pub_;
    ros::Publisher min_distance_pub_;
    ros::Publisher distance_info_pub_;
    
    // 参数
    std::string target_frame_;
    std::string powerline_topic_;
    std::string downsampled_topic_;
    
    // 障碍物检测参数
    float search_radius_;           // 电力线周围搜索半径
    float min_obstacle_distance_;   // 最小障碍物距离阈值
    float cluster_tolerance_;       // 聚类容差
    int min_cluster_size_;         // 最小聚类大小
    int max_cluster_size_;         // 最大聚类大小
    float min_obstacle_height_;    // 最小障碍物高度
    float max_obstacle_height_;    // 最大障碍物高度

    // 新增电力线排除参数
    float powerline_exclusion_radius_;     // 电力线排除半径
    float min_obstacle_separation_;        // 障碍物与电力线的最小分离距离
    bool use_intensity_filter_;            // 是否使用强度过滤
    float powerline_intensity_threshold_;  // 电力线强度阈值
    
    // 可视化参数
    bool enable_visualization_;
    bool show_distance_lines_;
    bool show_text_labels_;
    float marker_lifetime_;
    
    // 数据存储
    pcl::PointCloud<pcl::PointXYZI>::Ptr powerline_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud_;
    
    std::vector<ObstacleInfo> detected_obstacles_;
    float global_min_distance_;
    Eigen::Vector3f closest_powerline_point_;
    Eigen::Vector3f closest_obstacle_point_;
    
    // KD树用于快速搜索
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr powerline_kdtree_;
    
    // 可视化标记
    visualization_msgs::MarkerArray marker_array_;
    
    // 处理标志
    bool powerline_received_;
    bool downsampled_received_;
    
    // 统计信息
    ros::Time last_process_time_;
    int frame_count_;
};

#endif // OBSTACLE_DETECTOR_H


