
#include "matrix_transformer.h"


//pcl_conversions pcl_ros roscpp sensor_msgs std_msgs tf

pcl::PointCloud<pcl::PointXYZI>::Ptr
    pointCloufPtr(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    heightpointCloufPtr(new pcl::PointCloud<pcl::PointXYZI>());  //筛选高点云
pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;    //体素滤波
pcl::PointCloud<pcl::PointXYZI>::Ptr
laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());  //降采样后的点云

float minZ = 1000;
   
void MatrixTransformer::centerPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    // 计算点云的最小值和最大值
    pcl::PointXYZI min_point, max_point;
    pcl::getMinMax3D(*cloud, min_point, max_point);
    // 计算点云的中心
    Eigen::Vector4f centroid;
    centroid[0] = (min_point.x + max_point.x) / 2;
    centroid[1] = (min_point.y + max_point.y) / 2;
    centroid[2] = (min_point.z + max_point.z) / 2;
    centroid[3] = 1.0f;  // 维持一致性，通常设置为 1
    // 平移点云，使得中心为原点 (0, 0, 0)
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x -= centroid[0];
        cloud->points[i].y -= centroid[1];
        cloud->points[i].z -= centroid[2];
    }  
}

MatrixTransformer::MatrixTransformer() : nh_("~"), point_cloud_data_(nullptr), num_points_(0) {
    // Read parameters
    nh_.param<double>("scale_factor", scale_factor_, 1.0);
    nh_.param<std::string>("transform_type", transform_type_, "none");
    nh_.param<std::string>("data_folder", mat_file_path_, "");
    nh_.param<double>("scanVoxelSize", scanVoxelSize, 0.5);

    // Check parameters
    checkParameters();

    // Initialize publisher
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
    test_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("test", 1);

    // Load .mat file
    if (!loadMatFile(mat_file_path_)) {
        ROS_ERROR("Failed to load .mat file: %s, shutting down", mat_file_path_.c_str());
        ros::shutdown();
    }
}

MatrixTransformer::~MatrixTransformer() {
    // Free memory
    if (point_cloud_data_) {
        for (size_t i = 0; i < num_points_; ++i) {
            delete[] point_cloud_data_[i];
        }
        delete[] point_cloud_data_;
    }
}
/// Check parameters
void MatrixTransformer::checkParameters() {
    ROS_INFO("Parameter 'scale_factor': %f", scale_factor_);
    ROS_INFO("Parameter 'transform_type': %s", transform_type_.c_str());
    ROS_INFO("Parameter 'data_folder': %s", mat_file_path_.c_str());

    if (scale_factor_ <= 0) {
        ROS_WARN("scale_factor is invalid (<= 0), using default value 1.0");
        scale_factor_ = 1.0;
    }

    if (transform_type_ != "rotate" && transform_type_ != "scale" && transform_type_ != "none") {
        ROS_WARN("transform_type is invalid, using default 'none'");
        transform_type_ = "none";
    }

    if (mat_file_path_.empty()) {
        ROS_ERROR("data_folder parameter is not set or empty");
        ros::shutdown();
    }
}
/// Load .mat file and read point cloud data 
bool MatrixTransformer::loadMatFile(const std::string& file_path) {
    // Open .mat file
    mat_t* matfp = Mat_Open(file_path.c_str(), MAT_ACC_RDONLY);
    if (!matfp) {
        ROS_ERROR("Cannot open .mat file: %s", file_path.c_str());
        return false;
    }

    // Read ptCloudA struct
    matvar_t* matvar = Mat_VarRead(matfp, "ptCloudA");
    if (!matvar || matvar->class_type != MAT_C_STRUCT) {
        ROS_ERROR("Failed to read ptCloudA struct");
        Mat_Close(matfp);
        return false;
    }

    // Get data field
    matvar_t* data_field = Mat_VarGetStructFieldByName(matvar, "data", 0);
    if (!data_field || data_field->class_type != MAT_C_DOUBLE) {
        ROS_ERROR("Failed to read ptCloudA.data field");
        Mat_VarFree(matvar);
        Mat_Close(matfp);
        return false;
    }

    // Get dimensions
    size_t* dims = data_field->dims;
    num_points_ = dims[0]; // 2000000
    size_t num_fields = dims[1]; // 10
    if (num_fields < 3) {
        ROS_ERROR("Data has fewer than 3 columns (x, y, z)");
        Mat_VarFree(matvar);
        Mat_Close(matfp);
        return false;
    }
    ROS_INFO("dims[0] =  %zu points from .mat file", num_points_);
    ROS_INFO("dims[1] =  %zu points from .mat file", num_fields);

    // Allocate memory
    point_cloud_data_ = new double*[num_points_];
    for (size_t i = 0; i < num_points_; ++i) {
        point_cloud_data_[i] = new double[3]; // Store x, y, z
    }

    // Copy x, y, z data
    double* data_ptr = static_cast<double*>(data_field->data);
    double test[2000000][10];

    for (size_t i = 0; i < num_points_; ++i) {
        point_cloud_data_[i][0] = data_ptr[0 * num_points_ + i] / scale_factor_; // x
        point_cloud_data_[i][1] = data_ptr[1 * num_points_ + i] / scale_factor_; // y
        point_cloud_data_[i][2] = data_ptr[2 * num_points_ + i]; // z
    }

    // // 显示前100个点的 x, y, z 坐标
    // for (size_t i = 0; i < 100 && i < num_points_; ++i) {
    // ROS_INFO("Point %zu: x = %f, y = %f, z = %f", i, point_cloud_data_[i][0], point_cloud_data_[i][1], point_cloud_data_[i][2]);
    // }

    ROS_INFO("Loaded %zu points from .mat file", num_points_);

    // Clean up
    Mat_VarFree(matvar);
    Mat_Close(matfp);
    return true;
}

/// Convert double data to point cloud   将double的数组转化为点云类型
void MatrixTransformer::doubleToPointCloud()
{

    pointCloufPtr->clear();
    pcl::PointXYZI point;
    for (size_t i = 0; i < num_points_; ++i) {
        point.x = point_cloud_data_[i][0] -320700+50; // x
        point.y = point_cloud_data_[i][1] -4783000-100; // y
        point.z = point_cloud_data_[i][2] -260; // z
       
        pointCloufPtr->push_back(point);
    }

    // normalizePointCloudZ();

    int laserCloudSize = pointCloufPtr->points.size();
    for (int i = 0; i < laserCloudSize; i++)
    {
        point = pointCloufPtr->points[i];
        float pointX = point.x;
        float pointY = point.y;
        float pointZ = point.z;

        if (1)
        {
            point.x = pointX;
            point.y = pointY;
            point.z = pointZ;
            point.intensity = 1.0;
            heightpointCloufPtr->push_back(point);
        }
        
    }
    
    downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);
    laserCloudDwz->clear();
    downSizeFilter.setInputCloud(heightpointCloufPtr);  //下降采样，输入要降采样的点云
    downSizeFilter.filter(*laserCloudDwz);  //输出降采样后的点云

    // centerPointCloud(laserCloudDwz);
    for (size_t i = 0; i < 100 && i < num_points_; ++i) 
    {
        ROS_INFO("heightpointCloufPtr %zu: x = %f, y = %f, z = %f", i, heightpointCloufPtr->points[i].x,heightpointCloufPtr->points[i].y, heightpointCloufPtr->points[i].z);
    }
}

//寻找最小的z值
void MatrixTransformer::normalizePointCloudZ()
{
    // Step 1: Find the minimum z value
    

    // Iterate over each point in the point cloud
    for (size_t i = 0; i < pointCloufPtr->points.size(); ++i)
    {
        if (pointCloufPtr->points[i].z < minZ)
        {
            minZ = pointCloufPtr->points[i].z;
        }
    }

    // Step 2: Subtract the minimum z value from each point's z value
    for (size_t i = 0; i < pointCloufPtr->points.size(); ++i)
    {
        pointCloufPtr->points[i].x -= 320700-50;
        pointCloufPtr->points[i].y -= 4783100;
        pointCloufPtr->points[i].z -= minZ;
    }

    // The cloud is now normalized such that the lowest z value is at zero
}


//发布点云数据
void MatrixTransformer::publishPointCloud() {
    if (num_points_ == 0) {
        ROS_WARN("No point cloud data to publish");
        return;
    }

    doubleToPointCloud();

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*laserCloudDwz, cloud_msg);
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = ros::Time::now();

    // pcl::PointCloud<pcl::PointXYZ> cloud;

    // pcl::fromROSMsg(cloud_msg, cloud);
    // // 2. 输出点云的第一个点的坐标，检查转换是否成功
    // if (cloud.points.size() > 0) {
    //     ROS_INFO("First point: x = %f, y = %f, z = %f", cloud.points[0].x, cloud.points[0].y, cloud.points[0].z);
    // } else {
    //     ROS_WARN("PointCloud is empty");
    // }
    // // 3. 遍历输出部分点的坐标
    // for (size_t i = 0; i < 100 && i < num_points_; ++i) {  // 输出前 10 个点
    //     ROS_INFO("Point %zu: x = %f, y = %f, z = %f", i, cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
    // }

    
    //convertToPointCloud2(cloud_msg);

    cloud_pub_.publish(cloud_msg);
    
    ROS_INFO("Published point cloud with %zu points", num_points_);
}


void MatrixTransformer::testpublishPointCloud()
{
    sensor_msgs::PointCloud2 cloud_msg;
    // 设置消息类型和 frame_id（坐标系）
    cloud_msg.header.frame_id = "map";  // 设置正确的坐标系
    cloud_msg.height = 1;  // 点云为 1 行
    cloud_msg.width = 100;  // 假设我们有 100 个点
    cloud_msg.is_dense = true;
    // 填充点云数据（简单示例）
    cloud_msg.data.resize(100 * sizeof(float) * 4);  // 假设每个点有 4 个 float（x, y, z, intensity）
    for (int i = 0; i < 100; ++i)
    {
        cloud_msg.data[i * 16] = i * 0.1;  // x
        cloud_msg.data[i * 16 + 4] = i * 0.1;  // y
        cloud_msg.data[i * 16 + 8] = i * 0.1;  // z
        cloud_msg.data[i * 16 + 12] = 1.0;  // intensity
    }
    // 发布点云消息
    test_pub_.publish(cloud_msg);
}