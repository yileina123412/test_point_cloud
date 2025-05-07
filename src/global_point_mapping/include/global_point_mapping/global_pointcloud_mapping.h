#ifndef GLOBAL_POINTCLOUD_MAPPING_H
#define GLOBAL_POINTCLOUD_MAPPING_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
// 不使用 crop_sphere.h 库
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <mutex>
#include <set>
#include <map>

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudT;

/**
 * @brief 网格坐标的比较函数
 */
struct VoxelKeyCompare {
    bool operator()(const Eigen::Vector3i& k1, const Eigen::Vector3i& k2) const {
        if (k1.x() != k2.x()) return k1.x() < k2.x();
        if (k1.y() != k2.y()) return k1.y() < k2.y();
        return k1.z() < k2.z();
    }
};

/**
 * @brief 存储在网格中的点云信息
 */
struct VoxelData {
    std::vector<PointType> points;       // 网格中的点
    std::vector<float> weights;          // 每个点的权重
    ros::Time latest_timestamp;          // 最近更新时间
    bool in_local_range;                 // 是否在当前局部范围内
};

/**
 * @brief 全局点云构图类，基于空间网格的时间戳管理
 */
class GlobalPointcloudMapping {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    GlobalPointcloudMapping(ros::NodeHandle& nh);
    
    /**
     * @brief 析构函数
     */
    ~GlobalPointcloudMapping();
    
    /**
     * @brief 初始化参数和订阅者、发布者
     */
    bool init();
    
    /**
     * @brief 运行主循环
     */
    void run();

private:
    /**
     * @brief 点云回调函数
     * @param cloud_msg 接收到的点云消息
     */
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    
    /**
     * @brief IMU回调函数
     * @param imu_msg 接收到的IMU消息
     */
    void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg);
    
    /**
     * @brief 更新全局点云地图
     * @param cloud_in 输入的当前帧点云
     * @param transform 当前点云相对于全局坐标系的变换
     */
    void updateGlobalMap(const PointCloudT::Ptr& cloud_in, const Eigen::Matrix4f& transform);
    
    /**
     * @brief 根据IMU数据更新变换矩阵
     * @return 返回最新的变换矩阵
     */
    Eigen::Matrix4f updateTransformFromIMU();
    
    /**
     * @brief 裁剪点云，只保留以雷达为原点一定半径内的点
     * @param cloud_in 输入点云
     * @param cloud_out 输出点云
     * @param radius 半径范围
     */
    void cropPointsWithinRadius(const PointCloudT::Ptr& cloud_in, PointCloudT::Ptr& cloud_out, float radius);
    
    /**
     * @brief 根据点云计算出在局部范围内的网格坐标集合
     * @param cloud_in 输入点云
     * @param voxel_size 网格大小
     * @return 网格坐标集合
     */
    std::set<Eigen::Vector3i, VoxelKeyCompare> getVoxelsInLocalRange(
        const PointCloudT::Ptr& cloud_in, float voxel_size);
    
    /**
     * @brief 将点转换为网格坐标
     * @param point 点坐标
     * @param voxel_size 网格大小
     * @return 网格坐标
     */
    Eigen::Vector3i getVoxelCoord(const PointType& point, float voxel_size);
    
    /**
     * @brief 根据权重更新网格中的点
     * @param voxel 网格数据
     * @param new_point 新点
     * @param current_stamp 当前时间戳
     * @param time_weight_factor 时间权重因子
     */
    void updateVoxelWithWeights(
        VoxelData& voxel, 
        const PointType& new_point, 
        const ros::Time& current_stamp,
        float time_weight_factor);
    
    /**
     * @brief 从网格数据重建全局点云
     */
    void rebuildGlobalMapFromVoxels();
    
    /**
     * @brief 发布全局点云地图
     */
    void publishGlobalMap();
    
    /**
     * @brief 清理过期网格
     * @param current_time 当前时间
     * @param clean_threshold 清理时间阈值（秒）
     */
    void cleanOldVoxels(const ros::Time& current_time, double clean_threshold);

    // ROS相关
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber imu_sub_;
    ros::Publisher global_map_pub_;
    tf::TransformBroadcaster tf_broadcaster_;
    
    // 参数
    std::string cloud_topic_;
    std::string imu_topic_;
    std::string global_frame_id_;
    std::string lidar_frame_id_;
    float voxel_size_;                // 空间网格大小
    float crop_radius_;               // 裁剪半径
    double point_weight_decay_;       // 点权重衰减系数
    double voxel_clean_threshold_;    // 网格清理阈值（秒）
    bool first_frame_;                // 是否为第一帧
    
    // 数据
    PointCloudT::Ptr global_map_;    // 全局点云地图
    std::map<Eigen::Vector3i, VoxelData, VoxelKeyCompare> voxel_map_; // 网格地图
    std::set<Eigen::Vector3i, VoxelKeyCompare> current_local_voxels_; // 当前局部范围内的网格
    Eigen::Vector3f current_position_; // 当前位置
    Eigen::Matrix4f init_transform_;   // 初始变换矩阵
    Eigen::Matrix4f current_transform_; // 当前变换矩阵
    
    // 锁
    std::mutex map_mutex_;
    std::mutex imu_mutex_;
    
    // IMU数据
    sensor_msgs::Imu latest_imu_;
    bool has_imu_;
    bool has_cloud_;   // 是否接收到点云数据
    bool system_initialized_; // 系统是否初始化完成
    
    // 四元数和旋转矩阵
    Eigen::Quaternionf init_orientation_;
    
    // 点云处理工具
    pcl::VoxelGrid<PointType> voxel_filter_;
};

#endif // GLOBAL_POINTCLOUD_MAPPING_H