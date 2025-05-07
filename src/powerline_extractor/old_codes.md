### pca提取电力线 并进行聚类

#### powerline_extractor_node.cpp

```c++
#include "powerline_extractor.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "powerline_extractor_node");
    ros::NodeHandle nh;
    
    PowerlineExtractor extractor;
    
    // 设置1Hz的发布频率
    ros::Rate rate(1); // 1 Hz
    while (ros::ok()) {
        extractor.processAndPublishPowerlines();
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
```

#### powerline_extractor.cpp

```c++
#include "powerline_extractor.h"

PowerlineExtractor::PowerlineExtractor() 
    : nh_("~"), 
      point_cloud_data_(nullptr), 
      num_points_(0),
      original_cloud_(new pcl::PointCloud<pcl::PointXYZI>()),
      non_ground_cloud_(new pcl::PointCloud<pcl::PointXYZI>()),
      powerline_cloud_(new pcl::PointCloud<pcl::PointXYZI>()),
      clustered_powerline_cloud_(new pcl::PointCloud<pcl::PointXYZI>()) {
    
    // 读取参数
    nh_.param<double>("scale_factor", scale_factor_, 1.0);
    nh_.param<std::string>("data_folder", mat_file_path_, "");
    nh_.param<double>("voxel_size", voxel_size_, 0.05);
    nh_.param<double>("pca_radius", pca_radius_, 0.5);
    nh_.param<double>("angle_threshold", angle_threshold_, 10.0);
    nh_.param<double>("linearity_threshold", linearity_threshold_, 0.98);

    nh_.param<double>("cluster_tolerance", cluster_tolerance_, 2.0);
    nh_.param<int>("min_cluster_size", min_cluster_size_, 15);
    nh_.param<int>("max_cluster_size", max_cluster_size_, 100000);
    
    // 检查参数
    checkParameters();
    
    // 初始化发布器
    original_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("original_cloud", 1);
    non_ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("non_ground_cloud", 1);
    powerline_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("powerline_cloud", 1);
    clustered_powerline_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("clustered_powerline_cloud", 1);
    // 加载.mat文件
    if (!loadMatFile(mat_file_path_)) {
        ROS_ERROR("Failed to load .mat file: %s, shutting down", mat_file_path_.c_str());
        ros::shutdown();
    }
}

PowerlineExtractor::~PowerlineExtractor() {
    // 释放内存
    if (point_cloud_data_) {
        for (size_t i = 0; i < num_points_; ++i) {
            delete[] point_cloud_data_[i];
        }
        delete[] point_cloud_data_;
    }
}

void PowerlineExtractor::checkParameters() {
    ROS_INFO("Parameter 'scale_factor': %f", scale_factor_);
    ROS_INFO("Parameter 'data_folder': %s", mat_file_path_.c_str());
    ROS_INFO("Parameter 'voxel_size': %f", voxel_size_);
    ROS_INFO("Parameter 'pca_radius': %f", pca_radius_);
    ROS_INFO("Parameter 'angle_threshold': %f", angle_threshold_);
    ROS_INFO("Parameter 'linearity_threshold': %f", linearity_threshold_);

    ROS_INFO("Parameter 'cluster_tolerance': %f", cluster_tolerance_);
    ROS_INFO("Parameter 'min_cluster_size': %d", min_cluster_size_);
    ROS_INFO("Parameter 'max_cluster_size': %d", max_cluster_size_);
    
    if (scale_factor_ <= 0) {
        ROS_WARN("scale_factor is invalid (<= 0), using default value 1.0");
        scale_factor_ = 1.0;
    }
    
    if (mat_file_path_.empty()) {
        ROS_ERROR("data_folder parameter is not set or empty");
        ros::shutdown();
    }
}

bool PowerlineExtractor::loadMatFile(const std::string& file_path) {
    // 打开.mat文件
    mat_t* matfp = Mat_Open(file_path.c_str(), MAT_ACC_RDONLY);
    if (!matfp) {
        ROS_ERROR("Cannot open .mat file: %s", file_path.c_str());
        return false;
    }
    
    // 读取ptCloudA结构体
    matvar_t* matvar = Mat_VarRead(matfp, "ptCloudA");
    if (!matvar || matvar->class_type != MAT_C_STRUCT) {
        ROS_ERROR("Failed to read ptCloudA struct");
        Mat_Close(matfp);
        return false;
    }
    
    // 获取data字段
    matvar_t* data_field = Mat_VarGetStructFieldByName(matvar, "data", 0);
    if (!data_field || data_field->class_type != MAT_C_DOUBLE) {
        ROS_ERROR("Failed to read ptCloudA.data field");
        Mat_VarFree(matvar);
        Mat_Close(matfp);
        return false;
    }
    
    // 获取维度
    size_t* dims = data_field->dims;
    num_points_ = dims[0];
    size_t num_fields = dims[1];
    if (num_fields < 3) {
        ROS_ERROR("Data has fewer than 3 columns (x, y, z)");
        Mat_VarFree(matvar);
        Mat_Close(matfp);
        return false;
    }
    ROS_INFO("Loaded point cloud with dimensions: %zu x %zu", num_points_, num_fields);
    
    // 分配内存
    point_cloud_data_ = new double*[num_points_];
    for (size_t i = 0; i < num_points_; ++i) {
        point_cloud_data_[i] = new double[3]; // 存储x, y, z
    }
    
    // 复制x, y, z数据
    double* data_ptr = static_cast<double*>(data_field->data);
    for (size_t i = 0; i < num_points_; ++i) {
        point_cloud_data_[i][0] = data_ptr[0 * num_points_ + i] / scale_factor_; // x
        point_cloud_data_[i][1] = data_ptr[1 * num_points_ + i] / scale_factor_; // y
        point_cloud_data_[i][2] = data_ptr[2 * num_points_ + i]; // z
    }
    
    ROS_INFO("Loaded %zu points from .mat file", num_points_);
    
    // 清理
    Mat_VarFree(matvar);
    Mat_Close(matfp);
    return true;
}

void PowerlineExtractor::doubleToPointCloud() {
    original_cloud_->clear();
    pcl::PointXYZI point;
    
    for (size_t i = 0; i < num_points_; ++i) {
        point.x = point_cloud_data_[i][0] -320700+50 ;
        point.y = point_cloud_data_[i][1] -4783000-100 ;
        point.z = point_cloud_data_[i][2] -260 ;
        point.intensity = 1.0;
        original_cloud_->push_back(point);
    }
    
    ROS_INFO("Converted %zu points to PCL point cloud", original_cloud_->size());
}

void PowerlineExtractor::extractNonGroundPoints() {
    non_ground_cloud_->clear();
    
    // 使用直方图方法找到地面高度
    const int num_bins = 50;
    std::vector<int> histogram(num_bins, 0);
    
    // 找到z值的范围
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    
    for (const auto& point : original_cloud_->points) {
        min_z = std::min(min_z, point.z);
        max_z = std::max(max_z, point.z);
    }
    
    float bin_size = (max_z - min_z) / num_bins;
    
    // 构建高度直方图
    for (const auto& point : original_cloud_->points) {
        int bin_idx = std::min(static_cast<int>((point.z - min_z) / bin_size), num_bins - 1);
        histogram[bin_idx]++;
    }
    
    // 找到直方图中的最大值
    int max_count = 0;
    int max_bin_idx = 0;
    
    for (int i = 0; i < num_bins; i++) {
        if (histogram[i] > max_count) {
            max_count = histogram[i];
            max_bin_idx = i;
        }
    }
    
    // 在最大值上方3个bin设置地面阈值
    float ground_threshold = min_z + (max_bin_idx + 3) * bin_size;
    
    // 筛选非地面点
    for (const auto& point : original_cloud_->points) {
        if (point.z > ground_threshold) {
            non_ground_cloud_->push_back(point);
        }
    }
    
    ROS_INFO("Extracted %zu non-ground points", non_ground_cloud_->size());
}

void PowerlineExtractor::downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, 
                                             pcl::PointCloud<pcl::PointXYZI>::Ptr& output) {
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel_filter.setInputCloud(input);
    voxel_filter.filter(*output);
    
    ROS_INFO("Downsampled point cloud from %zu to %zu points", 
             input->size(), output->size());
}

void PowerlineExtractor::centerPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    pcl::PointXYZI min_point, max_point;
    pcl::getMinMax3D(*cloud, min_point, max_point);
    
    Eigen::Vector4f centroid;
    centroid[0] = (min_point.x + max_point.x) / 2;
    centroid[1] = (min_point.y + max_point.y) / 2;
    centroid[2] = (min_point.z + max_point.z) / 2;
    centroid[3] = 1.0f;
    
    for (auto& point : cloud->points) {
        point.x -= centroid[0];
        point.y -= centroid[1];
        point.z -= centroid[2];
    }
    
    ROS_INFO("Centered point cloud around (%.2f, %.2f, %.2f)", 
             centroid[0], centroid[1], centroid[2]);
}

void PowerlineExtractor::getPCA(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
                              pcl::PointCloud<pcl::PointXYZI>::Ptr& powerlinePoints) {
    powerlinePoints->clear();
    
    // 创建KD树用于近邻搜索
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud);
    
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    
    // 对每个点计算PCA
    for (size_t i = 0; i < cloud->size(); ++i) {
        // 搜索半径内的近邻点
        if (kdtree.radiusSearch(cloud->points[i], pca_radius_, 
                               pointIdxRadiusSearch, pointRadiusSquaredDistance) < 3) {
            continue; // 需要至少3个点进行PCA
        }
        
        // 将近邻点收集到一个矩阵中
        Eigen::MatrixXf neighborhood(pointIdxRadiusSearch.size(), 3);
        for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
            neighborhood(j, 0) = cloud->points[pointIdxRadiusSearch[j]].x;
            neighborhood(j, 1) = cloud->points[pointIdxRadiusSearch[j]].y;
            neighborhood(j, 2) = cloud->points[pointIdxRadiusSearch[j]].z;
        }
        
        // 计算近邻点的协方差矩阵
        Eigen::MatrixXf centered = neighborhood.rowwise() - neighborhood.colwise().mean();
        Eigen::MatrixXf cov = (centered.transpose() * centered) / float(neighborhood.rows() - 1);
        
        // 计算特征值和特征向量
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
        Eigen::Vector3f eigenvalues = eig.eigenvalues();
        Eigen::Matrix3f eigenvectors = eig.eigenvectors();
        
        // 确保特征值是降序排列的（由大到小）
        std::vector<std::pair<float, int>> eigenvalue_indices;
        for (int j = 0; j < 3; ++j) {
            eigenvalue_indices.push_back(std::make_pair(eigenvalues(j), j));
        }
        std::sort(eigenvalue_indices.begin(), eigenvalue_indices.end(), 
                 [](const std::pair<float, int>& a, const std::pair<float, int>& b) {
                     return a.first > b.first;
                 });
        
        // 获取排序后的特征值和特征向量
        float lambda1 = eigenvalues(eigenvalue_indices[0].second);
        float lambda2 = eigenvalues(eigenvalue_indices[1].second);
        float lambda3 = eigenvalues(eigenvalue_indices[2].second);
        Eigen::Vector3f normal = eigenvectors.col(eigenvalue_indices[0].second);
        
        // 计算线性度
        float linearity = (lambda1 - lambda2) / lambda1;
        
        // 计算法向量与垂直轴的夹角（度）
        float angle = std::acos(std::abs(normal(2)) / normal.norm()) * 180.0 / M_PI;
        
        // 检查是否是电力线的条件：接近水平且高度线性
        if (std::abs(angle - 90.0) < angle_threshold_ && linearity > linearity_threshold_) {
            powerlinePoints->push_back(cloud->points[i]);
        }
    }
    
    ROS_INFO("Extracted %zu powerline points using PCA", powerlinePoints->size());
}

void PowerlineExtractor::extractPowerlinePoints() {
    // 对非地面点进行下采样
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    downsamplePointCloud(non_ground_cloud_, downsampled_cloud);
    
    // 使用PCA提取电力线点
    getPCA(downsampled_cloud, powerline_cloud_);
}

void PowerlineExtractor::clusterPowerlines(const pcl::PointCloud<pcl::PointXYZI>::Ptr& candidateCloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& filteredCloud) {
if (candidateCloud->empty()) {
ROS_WARN("Candidate powerline cloud is empty, skipping clustering");
return;
}

filteredCloud->clear();

// 创建KD树用于Euclidean聚类
pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
tree->setInputCloud(candidateCloud);

// 存储聚类结果
std::vector<pcl::PointIndices> cluster_indices;

// 创建欧几里得聚类对象
pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
ec.setClusterTolerance(cluster_tolerance_);  // 设置聚类距离阈值
ec.setMinClusterSize(min_cluster_size_);     // 设置最小聚类点数
ec.setMaxClusterSize(max_cluster_size_);     // 设置最大聚类点数
ec.setSearchMethod(tree);
ec.setInputCloud(candidateCloud);
ec.extract(cluster_indices);

ROS_INFO("Found %zu clusters in candidate powerline points", cluster_indices.size());

// 处理每个聚类
int cluster_id = 0;
for (const auto& indices : cluster_indices) {
cluster_id++;

// 跳过太小的聚类（冗余检查）
if (indices.indices.size() < min_cluster_size_) {
continue;
}

// 提取当前聚类的点
for (const auto& index : indices.indices) {
pcl::PointXYZI point = candidateCloud->points[index];
// 可以设置intensity值为聚类ID，便于可视化不同的聚类
point.intensity = static_cast<float>(cluster_id);
filteredCloud->push_back(point);
}
}

ROS_INFO("Extracted %zu points from %zu clusters", filteredCloud->size(), cluster_indices.size());
}


void PowerlineExtractor::processAndPublishPowerlines() {
    // 步骤1：转换数据为点云
    doubleToPointCloud();
    
    // 发布原始点云
    sensor_msgs::PointCloud2 original_cloud_msg;
    pcl::toROSMsg(*original_cloud_, original_cloud_msg);
    original_cloud_msg.header.frame_id = "map";
    original_cloud_msg.header.stamp = ros::Time::now();
    original_cloud_pub_.publish(original_cloud_msg);
    
    // 步骤2：提取非地面点
    extractNonGroundPoints();
    
    // 发布非地面点云
    sensor_msgs::PointCloud2 non_ground_msg;
    pcl::toROSMsg(*non_ground_cloud_, non_ground_msg);
    non_ground_msg.header.frame_id = "map";
    non_ground_msg.header.stamp = ros::Time::now();
    non_ground_pub_.publish(non_ground_msg);
    
    // 步骤3：提取电力线点
    extractPowerlinePoints();
    
    // 发布电力线点云
    sensor_msgs::PointCloud2 powerline_msg;
    pcl::toROSMsg(*powerline_cloud_, powerline_msg);
    powerline_msg.header.frame_id = "map";
    powerline_msg.header.stamp = ros::Time::now();
    powerline_pub_.publish(powerline_msg);

    // 步骤4：对电力线点进行聚类
    clusterPowerlines(powerline_cloud_, clustered_powerline_cloud_);
    
    // 发布聚类后的电力线点云
    sensor_msgs::PointCloud2 clustered_powerline_msg;
    pcl::toROSMsg(*clustered_powerline_cloud_, clustered_powerline_msg);
    clustered_powerline_msg.header.frame_id = "map";
    clustered_powerline_msg.header.stamp = ros::Time::now();
    clustered_powerline_pub_.publish(clustered_powerline_msg);
    
    ROS_INFO("Processed and published point clouds: original(%zu), non-ground(%zu), powerline(%zu)",
             original_cloud_->size(), non_ground_cloud_->size(), powerline_cloud_->size());
}
```

#### powerline_extractor.h

```c++
#ifndef POWERLINE_EXTRACTOR_H
#define POWERLINE_EXTRACTOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <matio.h>
#include <string>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Dense>

class PowerlineExtractor {
public:
    PowerlineExtractor();
    ~PowerlineExtractor();
    void processAndPublishPowerlines();

private:
    // 加载.mat文件和基本处理
    void checkParameters();
    bool loadMatFile(const std::string& file_path);
    void doubleToPointCloud();
    
    // 电力线提取相关方法
    void extractNonGroundPoints();
    void extractPowerlinePoints();
    void getPCA(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
               pcl::PointCloud<pcl::PointXYZI>::Ptr& powerlinePoints);
    void clusterPowerlines(const pcl::PointCloud<pcl::PointXYZI>::Ptr& candidateCloud,
                pcl::PointCloud<pcl::PointXYZI>::Ptr& filteredCloud);
    
    // 其他工具方法
    void centerPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    void downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, 
                             pcl::PointCloud<pcl::PointXYZI>::Ptr& output);

    // ROS相关
    ros::NodeHandle nh_;
    ros::Publisher original_cloud_pub_;
    ros::Publisher non_ground_pub_;
    ros::Publisher powerline_pub_;

    ros::Publisher clustered_powerline_pub_;
    
    // 参数
    double scale_factor_;
    std::string mat_file_path_;
    double voxel_size_;
    double pca_radius_;
    double angle_threshold_;
    double linearity_threshold_;

    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    
    // 数据
    double** point_cloud_data_;
    size_t num_points_;
    
    // 点云对象
    pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr non_ground_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr powerline_cloud_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr clustered_powerline_cloud_;
    
    // 滤波器
    pcl::VoxelGrid<pcl::PointXYZI> down_size_filter_;
};

#endif // POWERLINE_EXTRACTOR_H
```

#### powerline_extraction.launch

```bash
<?xml version="1.0" encoding="UTF-8"?>
<launch>
  <!-- 参数设置 -->
  <arg name="mat_file_path" default="$(find powerline_extractor)/datas/L037_Sens1_600x250_cloud_8.mat"/>
  <arg name="voxel_size" default="0.05"/>
  <arg name="pca_radius" default="0.5"/>
  <arg name="angle_threshold" default="10.0"/>
  <arg name="linearity_threshold" default="0.98"/>

  <arg name="cluster_tolerance" default="2.0"/>
  <arg name="min_cluster_size" default="15"/>
  <arg name="max_cluster_size" default="100000"/>
  
  <!-- 启动电力线提取节点 -->
  <node name="powerline_extractor" pkg="powerline_extractor" type="powerline_extractor_node" output="screen">
    <param name="data_folder" value="$(arg mat_file_path)"/>
    <param name="voxel_size" value="$(arg voxel_size)"/>
    <param name="pca_radius" value="$(arg pca_radius)"/>
    <param name="angle_threshold" value="$(arg angle_threshold)"/>
    <param name="linearity_threshold" value="$(arg linearity_threshold)"/>

    <param name="cluster_tolerance" value="$(arg cluster_tolerance)"/>
    <param name="min_cluster_size" value="$(arg min_cluster_size)"/>
    <param name="max_cluster_size" value="$(arg max_cluster_size)"/>
  </node>
  
  <!-- 可视化 -->
  <node name="rviz" pkg="rviz" type="rviz" />
</launch>
```

### 添加聚类

#### powerline_extractor_node.cpp



#### powerline_extractor.cpp

```c++
#include "powerline_extractor.h"

PowerlineExtractor::PowerlineExtractor() 
    : nh_("~"), 
      point_cloud_data_(nullptr), 
      num_points_(0),
      original_cloud_(new pcl::PointCloud<pcl::PointXYZI>()),
      non_ground_cloud_(new pcl::PointCloud<pcl::PointXYZI>()),
      powerline_cloud_(new pcl::PointCloud<pcl::PointXYZI>()),
      clustered_powerline_cloud_(new pcl::PointCloud<pcl::PointXYZI>()) {
    
    // 读取参数
    nh_.param<double>("scale_factor", scale_factor_, 1.0);
    nh_.param<std::string>("data_folder", mat_file_path_, "");
    nh_.param<double>("voxel_size", voxel_size_, 0.05);
    nh_.param<double>("pca_radius", pca_radius_, 0.5);
    nh_.param<double>("angle_threshold", angle_threshold_, 10.0);
    nh_.param<double>("linearity_threshold", linearity_threshold_, 0.98);
    nh_.param<double>("cluster_tolerance", cluster_tolerance_, 2.0);
    nh_.param<int>("min_cluster_size", min_cluster_size_, 15);
    nh_.param<int>("max_cluster_size", max_cluster_size_, 100000);
    
    // 检查参数
    checkParameters();
    
    // 初始化发布器
    original_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("original_cloud", 1);
    non_ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("non_ground_cloud", 1);
    powerline_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("powerline_cloud", 1);
    clustered_powerline_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("clustered_powerline_cloud", 1);
    
    // 加载.mat文件
    if (!loadMatFile(mat_file_path_)) {
        ROS_ERROR("Failed to load .mat file: %s, shutting down", mat_file_path_.c_str());
        ros::shutdown();
    }
}

PowerlineExtractor::~PowerlineExtractor() {
    // 释放内存
    if (point_cloud_data_) {
        for (size_t i = 0; i < num_points_; ++i) {
            delete[] point_cloud_data_[i];
        }
        delete[] point_cloud_data_;
    }
}

void PowerlineExtractor::checkParameters() {
    ROS_INFO("Parameter 'scale_factor': %f", scale_factor_);
    ROS_INFO("Parameter 'data_folder': %s", mat_file_path_.c_str());
    ROS_INFO("Parameter 'voxel_size': %f", voxel_size_);
    ROS_INFO("Parameter 'pca_radius': %f", pca_radius_);
    ROS_INFO("Parameter 'angle_threshold': %f", angle_threshold_);
    ROS_INFO("Parameter 'linearity_threshold': %f", linearity_threshold_);
    ROS_INFO("Parameter 'cluster_tolerance': %f", cluster_tolerance_);
    ROS_INFO("Parameter 'min_cluster_size': %d", min_cluster_size_);
    ROS_INFO("Parameter 'max_cluster_size': %d", max_cluster_size_);
    
    if (scale_factor_ <= 0) {
        ROS_WARN("scale_factor is invalid (<= 0), using default value 1.0");
        scale_factor_ = 1.0;
    }
    
    if (mat_file_path_.empty()) {
        ROS_ERROR("data_folder parameter is not set or empty");
        ros::shutdown();
    }
}

bool PowerlineExtractor::loadMatFile(const std::string& file_path) {
    // 打开.mat文件
    mat_t* matfp = Mat_Open(file_path.c_str(), MAT_ACC_RDONLY);
    if (!matfp) {
        ROS_ERROR("Cannot open .mat file: %s", file_path.c_str());
        return false;
    }
    
    // 读取ptCloudA结构体
    matvar_t* matvar = Mat_VarRead(matfp, "ptCloudA");
    if (!matvar || matvar->class_type != MAT_C_STRUCT) {
        ROS_ERROR("Failed to read ptCloudA struct");
        Mat_Close(matfp);
        return false;
    }
    
    // 获取data字段
    matvar_t* data_field = Mat_VarGetStructFieldByName(matvar, "data", 0);
    if (!data_field || data_field->class_type != MAT_C_DOUBLE) {
        ROS_ERROR("Failed to read ptCloudA.data field");
        Mat_VarFree(matvar);
        Mat_Close(matfp);
        return false;
    }
    
    // 获取维度
    size_t* dims = data_field->dims;
    num_points_ = dims[0];
    size_t num_fields = dims[1];
    if (num_fields < 3) {
        ROS_ERROR("Data has fewer than 3 columns (x, y, z)");
        Mat_VarFree(matvar);
        Mat_Close(matfp);
        return false;
    }
    ROS_INFO("Loaded point cloud with dimensions: %zu x %zu", num_points_, num_fields);
    
    // 分配内存
    point_cloud_data_ = new double*[num_points_];
    for (size_t i = 0; i < num_points_; ++i) {
        point_cloud_data_[i] = new double[3]; // 存储x, y, z
    }
    
    // 复制x, y, z数据
    double* data_ptr = static_cast<double*>(data_field->data);
    for (size_t i = 0; i < num_points_; ++i) {
        point_cloud_data_[i][0] = data_ptr[0 * num_points_ + i] / scale_factor_; // x
        point_cloud_data_[i][1] = data_ptr[1 * num_points_ + i] / scale_factor_; // y
        point_cloud_data_[i][2] = data_ptr[2 * num_points_ + i]; // z
    }
    
    ROS_INFO("Loaded %zu points from .mat file", num_points_);
    
    // 清理
    Mat_VarFree(matvar);
    Mat_Close(matfp);
    return true;
}

void PowerlineExtractor::doubleToPointCloud() {
    original_cloud_->clear();
    pcl::PointXYZI point;
    
    // 首先读取所有点到点云中
    for (size_t i = 0; i < num_points_; ++i) {
        point.x = point_cloud_data_[i][0];
        point.y = point_cloud_data_[i][1];
        point.z = point_cloud_data_[i][2];
        point.intensity = 1.0;
        original_cloud_->push_back(point);
    }
    
    // 记录原始点云边界
    pcl::PointXYZI min_point, max_point;
    pcl::getMinMax3D(*original_cloud_, min_point, max_point);
    ROS_INFO("Original point cloud bounds: X[%.2f, %.2f], Y[%.2f, %.2f], Z[%.2f, %.2f]", 
             min_point.x, max_point.x, min_point.y, max_point.y, min_point.z, max_point.z);
    
    // 调整点云坐标
    adjustPointCloudOrigin(original_cloud_);
    
    // 获取调整后的边界以确认调整效果
    pcl::getMinMax3D(*original_cloud_, min_point, max_point);
    ROS_INFO("Adjusted point cloud bounds: X[%.2f, %.2f], Y[%.2f, %.2f], Z[%.2f, %.2f]", 
             min_point.x, max_point.x, min_point.y, max_point.y, min_point.z, max_point.z);
    
    ROS_INFO("Converted %zu points to PCL point cloud", original_cloud_->size());
}

void PowerlineExtractor::adjustPointCloudOrigin(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    // 找到点云的边界
    pcl::PointXYZI min_point, max_point;
    pcl::getMinMax3D(*cloud, min_point, max_point);
    
    // 计算中心点和最小Z值
    float center_x = (min_point.x + max_point.x) / 2.0f;
    float center_y = (min_point.y + max_point.y) / 2.0f;
    float min_z = min_point.z;
    
    ROS_INFO("Adjusting point cloud origin: Center(%.2f, %.2f), Min Z: %.2f", 
             center_x, center_y, min_z);
    
    // 将点云平移到中心
    for (auto& pt : cloud->points) {
        pt.x -= center_x;
        pt.y -= center_y;
        pt.z -= min_z;
    }
}

void PowerlineExtractor::extractNonGroundPoints() {
    non_ground_cloud_->clear();
    
    // 使用直方图方法找到地面高度
    const int num_bins = 50;
    std::vector<int> histogram(num_bins, 0);
    
    // 找到z值的范围
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    
    for (const auto& point : original_cloud_->points) {
        min_z = std::min(min_z, point.z);
        max_z = std::max(max_z, point.z);
    }
    
    float bin_size = (max_z - min_z) / num_bins;
    
    // 构建高度直方图
    for (const auto& point : original_cloud_->points) {
        int bin_idx = std::min(static_cast<int>((point.z - min_z) / bin_size), num_bins - 1);
        histogram[bin_idx]++;
    }
    
    // 找到直方图中的最大值
    int max_count = 0;
    int max_bin_idx = 0;
    
    for (int i = 0; i < num_bins; i++) {
        if (histogram[i] > max_count) {
            max_count = histogram[i];
            max_bin_idx = i;
        }
    }
    
    // 在最大值上方3个bin设置地面阈值
    float ground_threshold = min_z + (max_bin_idx + 3) * bin_size;
    
    // 筛选非地面点
    for (const auto& point : original_cloud_->points) {
        if (point.z > ground_threshold) {
            non_ground_cloud_->push_back(point);
        }
    }
    
    ROS_INFO("Extracted %zu non-ground points", non_ground_cloud_->size());
}

void PowerlineExtractor::downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, 
                                             pcl::PointCloud<pcl::PointXYZI>::Ptr& output) {
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel_filter.setInputCloud(input);
    voxel_filter.filter(*output);
    
    ROS_INFO("Downsampled point cloud from %zu to %zu points", 
             input->size(), output->size());
}

void PowerlineExtractor::centerPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    pcl::PointXYZI min_point, max_point;
    pcl::getMinMax3D(*cloud, min_point, max_point);
    
    Eigen::Vector4f centroid;
    centroid[0] = (min_point.x + max_point.x) / 2;
    centroid[1] = (min_point.y + max_point.y) / 2;
    centroid[2] = (min_point.z + max_point.z) / 2;
    centroid[3] = 1.0f;
    
    for (auto& point : cloud->points) {
        point.x -= centroid[0];
        point.y -= centroid[1];
        point.z -= centroid[2];
    }
    
    ROS_INFO("Centered point cloud around (%.2f, %.2f, %.2f)", 
             centroid[0], centroid[1], centroid[2]);
}

void PowerlineExtractor::getPCA(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
                              pcl::PointCloud<pcl::PointXYZI>::Ptr& powerlinePoints) {
    powerlinePoints->clear();
    
    // 创建KD树用于近邻搜索
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud);
    
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    
    // 对每个点计算PCA
    for (size_t i = 0; i < cloud->size(); ++i) {
        // 搜索半径内的近邻点
        if (kdtree.radiusSearch(cloud->points[i], pca_radius_, 
                               pointIdxRadiusSearch, pointRadiusSquaredDistance) < 3) {
            continue; // 需要至少3个点进行PCA
        }
        
        // 将近邻点收集到一个矩阵中
        Eigen::MatrixXf neighborhood(pointIdxRadiusSearch.size(), 3);
        for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
            neighborhood(j, 0) = cloud->points[pointIdxRadiusSearch[j]].x;
            neighborhood(j, 1) = cloud->points[pointIdxRadiusSearch[j]].y;
            neighborhood(j, 2) = cloud->points[pointIdxRadiusSearch[j]].z;
        }
        
        // 计算近邻点的协方差矩阵
        Eigen::MatrixXf centered = neighborhood.rowwise() - neighborhood.colwise().mean();
        Eigen::MatrixXf cov = (centered.transpose() * centered) / float(neighborhood.rows() - 1);
        
        // 计算特征值和特征向量
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
        Eigen::Vector3f eigenvalues = eig.eigenvalues();
        Eigen::Matrix3f eigenvectors = eig.eigenvectors();
        
        // 确保特征值是降序排列的（由大到小）
        std::vector<std::pair<float, int>> eigenvalue_indices;
        for (int j = 0; j < 3; ++j) {
            eigenvalue_indices.push_back(std::make_pair(eigenvalues(j), j));
        }
        std::sort(eigenvalue_indices.begin(), eigenvalue_indices.end(), 
                 [](const std::pair<float, int>& a, const std::pair<float, int>& b) {
                     return a.first > b.first;
                 });
        
        // 获取排序后的特征值和特征向量
        float lambda1 = eigenvalues(eigenvalue_indices[0].second);
        float lambda2 = eigenvalues(eigenvalue_indices[1].second);
        float lambda3 = eigenvalues(eigenvalue_indices[2].second);
        Eigen::Vector3f normal = eigenvectors.col(eigenvalue_indices[0].second);
        
        // 计算线性度
        float linearity = (lambda1 - lambda2) / lambda1;
        
        // 计算法向量与垂直轴的夹角（度）
        float angle = std::acos(std::abs(normal(2)) / normal.norm()) * 180.0 / M_PI;
        
        // 检查是否是电力线的条件：接近水平且高度线性
        if (std::abs(angle - 90.0) < angle_threshold_ && linearity > linearity_threshold_) {
            powerlinePoints->push_back(cloud->points[i]);
        }
    }
    
    ROS_INFO("Extracted %zu powerline points using PCA", powerlinePoints->size());
}

void PowerlineExtractor::extractPowerlinePoints() {
    // 对非地面点进行下采样
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    downsamplePointCloud(non_ground_cloud_, downsampled_cloud);
    
    // 使用PCA提取电力线点
    getPCA(downsampled_cloud, powerline_cloud_);
}

void PowerlineExtractor::clusterPowerlines(const pcl::PointCloud<pcl::PointXYZI>::Ptr& candidateCloud,
                                         pcl::PointCloud<pcl::PointXYZI>::Ptr& filteredCloud) {
    if (candidateCloud->empty()) {
        ROS_WARN("Candidate powerline cloud is empty, skipping clustering");
        return;
    }
    
    filteredCloud->clear();
    
    // 创建KD树用于Euclidean聚类
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(candidateCloud);
    
    // 存储聚类结果
    std::vector<pcl::PointIndices> cluster_indices;
    
    // 创建欧几里得聚类对象
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_tolerance_);  // 设置聚类距离阈值
    ec.setMinClusterSize(min_cluster_size_);     // 设置最小聚类点数
    ec.setMaxClusterSize(max_cluster_size_);     // 设置最大聚类点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(candidateCloud);
    ec.extract(cluster_indices);
    
    ROS_INFO("Found %zu clusters in candidate powerline points", cluster_indices.size());
    
    // 处理每个聚类
    int cluster_id = 0;
    for (const auto& indices : cluster_indices) {
        cluster_id++;
        
        // 跳过太小的聚类（冗余检查）
        if (indices.indices.size() < min_cluster_size_) {
            continue;
        }
        
        // 提取当前聚类的点
        for (const auto& index : indices.indices) {
            pcl::PointXYZI point = candidateCloud->points[index];
            // 可以设置intensity值为聚类ID，便于可视化不同的聚类
            point.intensity = static_cast<float>(cluster_id);
            filteredCloud->push_back(point);
        }
    }
    
    ROS_INFO("Extracted %zu points from %zu clusters", filteredCloud->size(), cluster_indices.size());
}

void PowerlineExtractor::processAndPublishPowerlines() {
    // 步骤1：转换数据为点云并调整坐标原点
    doubleToPointCloud();
    
    // 发布原始点云
    sensor_msgs::PointCloud2 original_cloud_msg;
    pcl::toROSMsg(*original_cloud_, original_cloud_msg);
    original_cloud_msg.header.frame_id = "map";
    original_cloud_msg.header.stamp = ros::Time::now();
    original_cloud_pub_.publish(original_cloud_msg);
    
    // 步骤2：提取非地面点
    extractNonGroundPoints();
    
    // 调整非地面点云坐标
    adjustPointCloudOrigin(non_ground_cloud_);
    
    // 发布非地面点云
    sensor_msgs::PointCloud2 non_ground_msg;
    pcl::toROSMsg(*non_ground_cloud_, non_ground_msg);
    non_ground_msg.header.frame_id = "map";
    non_ground_msg.header.stamp = ros::Time::now();
    non_ground_pub_.publish(non_ground_msg);
    
    // 步骤3：提取电力线点
    extractPowerlinePoints();
    
    // 发布电力线点云
    sensor_msgs::PointCloud2 powerline_msg;
    pcl::toROSMsg(*powerline_cloud_, powerline_msg);
    powerline_msg.header.frame_id = "map";
    powerline_msg.header.stamp = ros::Time::now();
    powerline_pub_.publish(powerline_msg);
    
    // 步骤4：对电力线点进行聚类
    clusterPowerlines(powerline_cloud_, clustered_powerline_cloud_);
    
    // 发布聚类后的电力线点云
    sensor_msgs::PointCloud2 clustered_powerline_msg;
    pcl::toROSMsg(*clustered_powerline_cloud_, clustered_powerline_msg);
    clustered_powerline_msg.header.frame_id = "map";
    clustered_powerline_msg.header.stamp = ros::Time::now();
    clustered_powerline_pub_.publish(clustered_powerline_msg);
    
    ROS_INFO("Processed and published point clouds: original(%zu), non-ground(%zu), powerline(%zu), clustered_powerline(%zu)",
             original_cloud_->size(), non_ground_cloud_->size(), powerline_cloud_->size(), clustered_powerline_cloud_->size());
}
```



#### powerline_extractor.h

```c++
#ifndef POWERLINE_EXTRACTOR_H
#define POWERLINE_EXTRACTOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <matio.h>
#include <string>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Dense>

class PowerlineExtractor {
public:
    PowerlineExtractor();
    ~PowerlineExtractor();
    void processAndPublishPowerlines();

private:
    // 加载.mat文件和基本处理
    void checkParameters();
    bool loadMatFile(const std::string& file_path);
    void doubleToPointCloud();
    
    // 电力线提取相关方法
    void extractNonGroundPoints();
    void extractPowerlinePoints();
    void getPCA(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
               pcl::PointCloud<pcl::PointXYZI>::Ptr& powerlinePoints);
    void clusterPowerlines(const pcl::PointCloud<pcl::PointXYZI>::Ptr& candidateCloud,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr& filteredCloud);
    
    // 其他工具方法
    void centerPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    void downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, 
                             pcl::PointCloud<pcl::PointXYZI>::Ptr& output);
    void adjustPointCloudOrigin(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

    // ROS相关
    ros::NodeHandle nh_;
    ros::Publisher original_cloud_pub_;
    ros::Publisher non_ground_pub_;
    ros::Publisher powerline_pub_;
    ros::Publisher clustered_powerline_pub_;
    
    // 参数
    double scale_factor_;
    std::string mat_file_path_;
    double voxel_size_;
    double pca_radius_;
    double angle_threshold_;
    double linearity_threshold_;
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    
    // 数据
    double** point_cloud_data_;
    size_t num_points_;
    
    // 点云对象
    pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr non_ground_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr powerline_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustered_powerline_cloud_;
    
    // 滤波器
    pcl::VoxelGrid<pcl::PointXYZI> down_size_filter_;
};

#endif // POWERLINE_EXTRACTOR_H
```



#### powerline_extraction.launch

```bash
<?xml version="1.0" encoding="UTF-8"?>
<launch>
  <!-- 参数设置 -->
  <arg name="mat_file_path" default="$(find powerline_extractor)/datas/L037_Sens1_600x250_cloud_8.mat"/>
  <arg name="voxel_size" default="0.05"/>
  <arg name="pca_radius" default="0.5"/>
  <arg name="angle_threshold" default="10.0"/>
  <arg name="linearity_threshold" default="0.98"/>
  <arg name="cluster_tolerance" default="2.0"/>
  <arg name="min_cluster_size" default="15"/>
  <arg name="max_cluster_size" default="100000"/>
  
  <!-- 启动电力线提取节点 -->
  <node name="powerline_extractor" pkg="powerline_extractor" type="powerline_extractor_node" output="screen">
    <param name="data_folder" value="$(arg mat_file_path)"/>
    <param name="voxel_size" value="$(arg voxel_size)"/>
    <param name="pca_radius" value="$(arg pca_radius)"/>
    <param name="angle_threshold" value="$(arg angle_threshold)"/>
    <param name="linearity_threshold" value="$(arg linearity_threshold)"/>
    <param name="cluster_tolerance" value="$(arg cluster_tolerance)"/>
    <param name="min_cluster_size" value="$(arg min_cluster_size)"/>
    <param name="max_cluster_size" value="$(arg max_cluster_size)"/>
  </node>
  
  <!-- 可视化 -->
  <node name="rviz" pkg="rviz" type="rviz" />
</launch>
```

