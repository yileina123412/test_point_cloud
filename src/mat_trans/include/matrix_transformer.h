#ifndef MATRIX_TRANSFORMER_H
#define MATRIX_TRANSFORMER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <matio.h>
#include <string>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>


class MatrixTransformer {
public:
    MatrixTransformer();
    ~MatrixTransformer();
    void publishPointCloud();
    void testpublishPointCloud();
    void normalizePointCloudZ();

private:
    void checkParameters();
    bool loadMatFile(const std::string& file_path);
    
    void doubleToPointCloud();
    void centerPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    

    ros::NodeHandle nh_;
    ros::Publisher cloud_pub_;
    ros::Publisher test_pub_;
    double scale_factor_;
    std::string transform_type_;
    std::string mat_file_path_;
    double** point_cloud_data_;
    size_t num_points_;
    double scanVoxelSize = 0.05;
};

#endif // MATRIX_TRANSFORMER_H
