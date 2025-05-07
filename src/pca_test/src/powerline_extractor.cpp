#include "powerline_extractor.h"
#include <pcl/common/common.h>
#include <vector>
#include <Eigen/Dense>


pcl::PointCloud<pcl::PointXYZI>::Ptr out_of_ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

PowerlineExtractor::PowerlineExtractor() : nh_("~") {
    sub_ = nh_.subscribe("/matrix_transformer_node/point_cloud", 1,
                         &PowerlineExtractor::pointCloudCallback, this);
   
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/out_rs_points", 1);
    ROS_INFO("Powerline Extractor Node Initialized, Subscribed to /matrix_transformer_node/point_cloud");
    isCallbackExecuted = false;
    flag1 = false;
}
void PowerlineExtractor::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
    try {
        ROS_INFO("Received point cloud: frame_id=%s, points=%u x %u",
                 input->header.frame_id.c_str(), input->width, input->height);
        
        if(!isCallbackExecuted)        
        {

            cloud->clear();

            pcl::fromROSMsg(*input, *cloud);
            if (cloud->empty()) {
                ROS_ERROR("Failed to convert PointCloud2 to PCL: empty cloud");
                return;
            }
            ROS_INFO("Converted to PCL cloud: %zu points", cloud->size());
            isCallbackExecuted = true;
            
        }
        
        
        

        
        
        
        // // 下采样
        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>);
        // pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        // voxel_filter.setInputCloud(cloud);
        // voxel_filter.setLeafSize(0.2f, 0.2f, 0.2f);
        // voxel_filter.filter(*cloud_downsampled);
        // ROS_INFO("Downsampled cloud: %zu points", cloud_downsampled->size());
        // pcl::PointCloud<pcl::PointXYZI>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        // ROS_INFO("Starting non-ground filtering...");
        // filterNonGroundPoints(cloud_downsampled, non_ground_cloud);
        // ROS_INFO("Non-ground filtering done: %zu points", non_ground_cloud->size());
        // pcl::PointCloud<pcl::PointXYZI>::Ptr powerline_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        // ROS_INFO("Starting powerline extraction...");
        // extractPowerLines(non_ground_cloud, powerline_cloud, 2.0, 10.0, 0.7);
        // ROS_INFO("Powerline extraction done: %zu points", powerline_cloud->size());

        // ROS_INFO("Published powerline cloud: %zu points", powerline_cloud->size());
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in callback: %s", e.what());
    }
}
//
void PowerlineExtractor::filterNonGroundPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr& outputCloud) {
        if (inputCloud->empty()) {
            ROS_INFO("Input cloud is empty, skipping filtering...");
            return;
        }
        outputCloud->clear();
        ROS_INFO("Filtering non-ground points from input cloud: %zu points", inputCloud->size());
        // 1. 计算 z 值的最小值和最大值
        pcl::PointXYZI min_point, max_point;
        pcl::getMinMax3D(*inputCloud, min_point, max_point);
        float minZ = min_point.z;
        float maxZ = max_point.z;
        // 2. 划分 z 值为 50 个区间
        int numIntervals = 50;
        float intervalSize = (maxZ - minZ) / numIntervals;
        std::vector<int> pointCounts(numIntervals, 0);
        // 3. 统计每个区间的点云数量
        for (const auto& point : inputCloud->points) {
            int index = static_cast<int>((point.z - minZ) / intervalSize);
            if (index >= 0 && index < numIntervals) {
                pointCounts[index]++;
            }
        }
        // 4. 找到点云最多的区间
        int maxCount = *std::max_element(pointCounts.begin(), pointCounts.end());
        int threshold = maxCount + 3;
        // 5. 过滤掉点云数量少于 maxCount + 3 的区间
        for (const auto& point : inputCloud->points) {
            int index = static_cast<int>((point.z - minZ) / intervalSize);
            if (index >= 0 && index < numIntervals && pointCounts[index] >= threshold) {
                outputCloud->points.push_back(point);
            }
        }
        // 设置输出点云的尺寸
        outputCloud->width = outputCloud->points.size();
        outputCloud->height = 1;  // 这里假设点云是无序的，所以高度设置为 1
        outputCloud->is_dense = true;
        outputCloud->header.frame_id = inputCloud->header.frame_id;
        flag1 = true;
        ROS_INFO("Filtered non-ground points: %zu points", outputCloud->size());
}
void PowerlineExtractor::extractPowerLines(pcl::PointCloud<pcl::PointXYZI>::Ptr non_ground_cloud,
                                          pcl::PointCloud<pcl::PointXYZI>::Ptr powerline_cloud,
                                          float radius, float angle_thr, float l_thr) {
    if (non_ground_cloud->empty()) {
        ROS_WARN("Empty non-ground cloud, skipping...");
        return;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr remaining_cloud = non_ground_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_powerline_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(radius);
    seg.setMaxIterations(500);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    int max_iterations = 100;
    int iteration = 0;
    while (remaining_cloud->size() > 100 && iteration < max_iterations) {
        seg.setInputCloud(remaining_cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            ROS_INFO("No inliers found, stopping...");
            break;
        }
        Eigen::Vector3f point_on_line(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        Eigen::Vector3f direction(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
        float max_deviation = 0.0;
        for (const auto& idx : inliers->indices) {
            Eigen::Vector3f p(remaining_cloud->points[idx].x,
                             remaining_cloud->points[idx].y,
                             remaining_cloud->points[idx].z);
            Eigen::Vector3f vec = p - point_on_line;
            Eigen::Vector3f cross = vec.cross(direction);
            float distance = cross.norm() / direction.norm();
            max_deviation = std::max(max_deviation, distance);
        }
        ROS_INFO("Iteration %d: Inliers found: %zu, Max deviation: %f", iteration, inliers->indices.size(), max_deviation);
        if (max_deviation / radius < (1.0 - l_thr) || inliers->indices.size() > 1000) {
            pcl::ExtractIndices<pcl::PointXYZI> extract;
            extract.setInputCloud(remaining_cloud);
            extract.setIndices(inliers);
            extract.setNegative(false);
            pcl::PointCloud<pcl::PointXYZI>::Ptr line_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            extract.filter(*line_cloud);
            *temp_powerline_cloud += *line_cloud;
        }
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(remaining_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        extract.filter(*temp_cloud);
        remaining_cloud = temp_cloud;
        iteration++;
    }
    *powerline_cloud = *temp_powerline_cloud;
}
void PowerlineExtractor::publichPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = cloud->header.frame_id;
    pub_.publish(output);
    ROS_INFO("Published powerline cloud: %zu points", cloud->size());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "powerline_extractor");

    ros::NodeHandle nh;  //创建对象nh，用于与ROS系统通信，进行话题的发布与接收
    ros::NodeHandle nh_private("~");  //访问节点私有参数  通常只存在于节点命名空间
    ros::Rate rate(2);

    PowerlineExtractor extractor;
    ROS_INFO("Node running, waiting for point cloud data...");
    
    // ros::spin();

    



    while(ros::ok())
    {
        ros::spinOnce();  //执行一次实践循环



   
        
        rate.sleep();     //让节点按照设定频率休眠   
    }

    
    return 0;
}