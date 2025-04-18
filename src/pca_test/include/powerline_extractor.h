
#ifndef POWERLINE_EXTRACTOR_H
#define POWERLINE_EXTRACTOR_H
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <vector>
#include <iostream>
#include <algorithm>

class PowerlineExtractor {
public:
    PowerlineExtractor();
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);
    void publichPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    bool flag1;
    bool isCallbackExecuted = false;

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    
    void filterNonGroundPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud, 
                            pcl::PointCloud<pcl::PointXYZI>::Ptr& outputCloud);
    void extractPowerLines(pcl::PointCloud<pcl::PointXYZI>::Ptr non_ground_cloud,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr powerline_cloud,
                          float radius, float angle_thr, float l_thr);
};
#endif // POWERLINE_EXTRACTOR_H