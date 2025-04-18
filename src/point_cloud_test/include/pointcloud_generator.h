#ifndef POINTCLOUD_GENERATOR_H
#define POINTCLOUD_GENERATOR_H
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>  // For getMinMax3D


class PointCloudGenerator {
public:
    PointCloudGenerator();
    void generateAndPublish();
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Publisher rspub_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud_;
    int flag = 1;
};
#endif // POINTCLOUD_GENERATOR_H