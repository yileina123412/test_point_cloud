#include "pointcloud_generator.h"
#include <random>
PointCloudGenerator::PointCloudGenerator() : nh_("~") {
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/random_pointcloud", 1);
    sub_ = nh_.subscribe("/rslidar_points", 1,
        &PointCloudGenerator::pointCloudCallback, this);
    rspub_ = nh_.advertise<sensor_msgs::PointCloud2>("/out_rslidar_points", 1);
    laserCloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    flag = 1;
}


void PointCloudGenerator::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
    
    ROS_INFO("Received point cloud: frame_id=%s, points=%u x %u",
        input->header.frame_id.c_str(), input->width, input->height);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*input, cloud);

    // 定义变量来保存最小值和最大值
    pcl::PointXYZI min_point, max_point;
    // 使用 pcl::getMinMax3D 计算点云的最小和最大值
    pcl::getMinMax3D(cloud, min_point, max_point);

    for(int i = 0; i < cloud.size(); i++)
    {
        if(cloud.points[i].x > 10.0 || cloud.points[i].x < -10.0)
        {
            cloud.points[i].x = 0;
            cloud.points[i].y = 0;
            cloud.points[i].z = 0;
        }
    }
    ROS_INFO("Cloud size = %lu max x =%f  minX = %f  ",cloud.size(),max_point.x,min_point.x);
    
        

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);

    rspub_.publish(output);


}
void PointCloudGenerator::generateAndPublish() {
    
    if(flag)
    {
        // Clear previous point cloud
    laserCloud_->clear();
    // Random number generation
    std::random_device rd;  //generate random number
    std::mt19937 gen(rd());   
    std::uniform_real_distribution<float> dist_pos(-10.0, 10.0);
    std::uniform_real_distribution<float> dist_intensity(0.0, 1.0);
    // Generate random points
    for (int i = 0; i < 1000; ++i) {
        pcl::PointXYZI point;
        point.x = dist_pos(gen);
        point.y = dist_pos(gen);
        point.z = dist_pos(gen);
        point.intensity = dist_intensity(gen);
        laserCloud_->push_back(point);
    }

        flag = 0;
    }
    
    // Convert to ROS message
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*laserCloud_, output);
    output.header.frame_id = "map";
    output.header.stamp = ros::Time::now();
    // Publish
    pub_.publish(output);
}
