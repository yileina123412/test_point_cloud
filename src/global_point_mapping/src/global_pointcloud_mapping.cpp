#include "global_point_mapping/global_pointcloud_mapping.h"

GlobalPointcloudMapping::GlobalPointcloudMapping(ros::NodeHandle& nh) : 
    nh_(nh),
    first_frame_(true),
    has_imu_(false),
    has_cloud_(false),
    system_initialized_(false),
    current_position_(Eigen::Vector3f::Zero())
{
    global_map_.reset(new PointCloudT());
}

GlobalPointcloudMapping::~GlobalPointcloudMapping() {
    // 释放资源
}

bool GlobalPointcloudMapping::init() {
    // 加载参数
    nh_.param<std::string>("cloud_topic", cloud_topic_, "/rslidar_points");
    nh_.param<std::string>("imu_topic", imu_topic_, "/rslidar_imu_data");
    nh_.param<std::string>("global_frame_id", global_frame_id_, "map");
    nh_.param<std::string>("lidar_frame_id", lidar_frame_id_, "rslidar");
    nh_.param<float>("voxel_size", voxel_size_, 0.1f);  // 网格大小，可调整
    nh_.param<float>("crop_radius", crop_radius_, 8.0f); // 裁剪半径
    nh_.param<double>("point_weight_decay", point_weight_decay_, 0.9); // 点权重衰减系数
    nh_.param<double>("voxel_clean_threshold", voxel_clean_threshold_, 300.0); // 5分钟未更新的网格才会被清理
    
    // 初始化变换矩阵为单位矩阵
    init_transform_ = Eigen::Matrix4f::Identity();
    current_transform_ = Eigen::Matrix4f::Identity();
    
    // 订阅点云和IMU数据
    cloud_sub_ = nh_.subscribe(cloud_topic_, 1, &GlobalPointcloudMapping::pointCloudCallback, this);
    imu_sub_ = nh_.subscribe(imu_topic_, 10, &GlobalPointcloudMapping::imuCallback, this);
    
    // 发布全局点云地图
    global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("global_map", 1);
    
    ROS_INFO("初始化改进版全局点云构图系统完成!");
    ROS_INFO("点云话题: %s", cloud_topic_.c_str());
    ROS_INFO("IMU话题: %s", imu_topic_.c_str());
    ROS_INFO("网格大小: %.2f", voxel_size_);
    ROS_INFO("局部范围半径: %.2f", crop_radius_);
    ROS_INFO("等待接收第一帧点云数据和IMU数据...");
    
    return true;
}

void GlobalPointcloudMapping::run() {
    ros::Rate rate(10); // 10Hz
    while (ros::ok()) {
        // 检查是否同时收到了点云数据和IMU数据
        if (!system_initialized_) {
            {
                std::lock_guard<std::mutex> lock_imu(imu_mutex_);
                std::lock_guard<std::mutex> lock_map(map_mutex_);
                
                if (has_cloud_ && has_imu_) {
                    system_initialized_ = true;
                    ROS_INFO("已接收到点云和IMU数据，系统开始运行!");
                }
            }
            
            // 如果系统未初始化，跳过本次循环
            if (!system_initialized_) {
                ROS_INFO("未收到点云和IMU数据，系统还未开始运行!");
                rate.sleep();
                continue;
            }
        }
        
        // 系统已初始化，执行正常处理
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            // 周期性清理过期网格
            cleanOldVoxels(ros::Time::now(), voxel_clean_threshold_);
            
            // 从网格数据重建全局点云
            rebuildGlobalMapFromVoxels();
        }
        
        // 发布全局点云地图
        publishGlobalMap();
        
        rate.sleep();
    }
}

void GlobalPointcloudMapping::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // 标记已接收到点云数据
    has_cloud_ = true;
    
    // 如果系统未初始化完成，只记录第一帧点云，不处理
    if (!system_initialized_ && first_frame_) {
        ROS_INFO("接收到第一帧点云数据");
        return;
    }
    
    // 将ROS消息转换为PCL点云
    PointCloudT::Ptr cloud_in(new PointCloudT());
    pcl::fromROSMsg(*cloud_msg, *cloud_in);
    
    // 裁剪点云，只保留半径范围内的点
    PointCloudT::Ptr cropped_cloud(new PointCloudT());
    cropPointsWithinRadius(cloud_in, cropped_cloud, crop_radius_);
    
    // 如果是第一帧，初始化世界坐标系
    if (first_frame_) {
        // 保存初始位姿
        if (has_imu_) {
            // 从IMU获取初始方向
            tf::Quaternion q(
                latest_imu_.orientation.x,
                latest_imu_.orientation.y,
                latest_imu_.orientation.z,
                latest_imu_.orientation.w
            );
            // 将z轴向上对齐
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            // 保持yaw，但将roll和pitch设置为0（使z轴向上）
            q = tf::createQuaternionFromRPY(0, 0, yaw);
            
            // 保存初始朝向
            init_orientation_ = Eigen::Quaternionf(q.w(), q.x(), q.y(), q.z());
            
            // 设置初始变换矩阵
            init_transform_.block<3,3>(0,0) = init_orientation_.toRotationMatrix();
        }
        first_frame_ = false;
        ROS_INFO("初始化世界坐标系完成!");
    }
    
    // 获取当前点云相对于世界坐标系的变换
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    if (has_imu_) {
        transform = updateTransformFromIMU();
    }
    
    // 计算当前局部范围内的网格
    current_local_voxels_ = getVoxelsInLocalRange(cropped_cloud, voxel_size_);
    
    // 更新全局地图
    updateGlobalMap(cropped_cloud, transform);
}

void GlobalPointcloudMapping::imuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    latest_imu_ = *imu_msg;
    
    // 标记已接收到IMU数据
    if (!has_imu_) {
        has_imu_ = true;
        ROS_INFO("接收到第一帧IMU数据");
    }
}

Eigen::Vector3i GlobalPointcloudMapping::getVoxelCoord(const PointType& point, float voxel_size) {
    int vx = std::floor(point.x / voxel_size);
    int vy = std::floor(point.y / voxel_size);
    int vz = std::floor(point.z / voxel_size);
    return Eigen::Vector3i(vx, vy, vz);
}

std::set<Eigen::Vector3i, VoxelKeyCompare> 
GlobalPointcloudMapping::getVoxelsInLocalRange(const PointCloudT::Ptr& cloud_in, float voxel_size) {
    std::set<Eigen::Vector3i, VoxelKeyCompare> local_voxels;
    
    for (const auto& point : cloud_in->points) {
        Eigen::Vector3i voxel_coord = getVoxelCoord(point, voxel_size);
        local_voxels.insert(voxel_coord);
    }
    
    return local_voxels;
}

void GlobalPointcloudMapping::updateVoxelWithWeights(
    VoxelData& voxel, 
    const PointType& new_point, 
    const ros::Time& current_stamp,
    float time_weight_factor) {
    
    // 计算新点的权重
    float new_weight = 1.0f;
    
    // 检查是否有类似的点
    bool point_added = false;
    const float distance_threshold = voxel_size_ * 0.3f; // 距离阈值，可调整
    
    for (size_t i = 0; i < voxel.points.size(); ++i) {
        PointType& existing_point = voxel.points[i];
        float& existing_weight = voxel.weights[i];
        
        // 计算点之间的距离
        float distance = std::sqrt(
            std::pow(existing_point.x - new_point.x, 2) +
            std::pow(existing_point.y - new_point.y, 2) +
            std::pow(existing_point.z - new_point.z, 2)
        );
        
        // 如果点很接近，根据权重更新
        if (distance < distance_threshold) {
            // 计算时间差(秒)
            double time_diff = (current_stamp - voxel.latest_timestamp).toSec();
            
            // 时间因子，时间差越大权重越小
            float time_factor = std::exp(-time_diff / time_weight_factor);
            
            // 对旧点应用衰减
            existing_weight *= point_weight_decay_;
            
            // 归一化权重
            float total_weight = existing_weight + new_weight;
            float w1 = existing_weight / total_weight;
            float w2 = new_weight / total_weight;
            
            // 加权融合点的坐标和强度
            existing_point.x = w1 * existing_point.x + w2 * new_point.x;
            existing_point.y = w1 * existing_point.y + w2 * new_point.y;
            existing_point.z = w1 * existing_point.z + w2 * new_point.z;
            existing_point.intensity = w1 * existing_point.intensity + w2 * new_point.intensity;
            
            // 更新权重
            existing_weight = total_weight;
            
            point_added = true;
            break;
        }
    }
    
    // 如果没有找到足够近的点，添加为新点
    if (!point_added) {
        voxel.points.push_back(new_point);
        voxel.weights.push_back(new_weight);
    }
    
    // 更新时间戳
    voxel.latest_timestamp = current_stamp;
    voxel.in_local_range = true;
}

void GlobalPointcloudMapping::updateGlobalMap(const PointCloudT::Ptr& cloud_in, const Eigen::Matrix4f& transform) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    // 将当前点云转换到全局坐标系
    PointCloudT::Ptr transformed_cloud(new PointCloudT());
    pcl::transformPointCloud(*cloud_in, *transformed_cloud, transform);
    
    // 当前点云的时间戳
    ros::Time current_stamp = ros::Time::now();
    
    // 更新当前位置
    current_position_ = Eigen::Vector3f(
        transform(0, 3), 
        transform(1, 3), 
        transform(2, 3)
    );
    
    // 重置所有网格的局部范围标志
    for (auto& voxel_pair : voxel_map_) {
        voxel_pair.second.in_local_range = false;
    }
    
    // 根据当前点云更新网格地图
    for (const auto& point : transformed_cloud->points) {
        Eigen::Vector3i voxel_coord = getVoxelCoord(point, voxel_size_);
        
        // 更新或创建网格
        auto& voxel = voxel_map_[voxel_coord];
        
        // 只有在局部范围内的网格才更新
        if (current_local_voxels_.find(voxel_coord) != current_local_voxels_.end()) {
            updateVoxelWithWeights(voxel, point, current_stamp, 1.0);
        }
    }
}

Eigen::Matrix4f GlobalPointcloudMapping::updateTransformFromIMU() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    // 设置静态变量，用于姿态积分
    static Eigen::Quaternionf estimated_orientation = Eigen::Quaternionf::Identity();
    static ros::Time last_orientation_time = ros::Time::now();
    static bool orientation_initialized = false;
    
    // 检查IMU提供的四元数是否有效
    bool valid_orientation = true;
    tf::Quaternion q(
        latest_imu_.orientation.x,
        latest_imu_.orientation.y,
        latest_imu_.orientation.z,
        latest_imu_.orientation.w
    );
    
    // 检查四元数是否为零或包含NaN值 
    if ((q.x() == 0 && q.y() == 0 && q.z() == 0 && q.w() == 0) ||
        std::isnan(q.x()) || std::isnan(q.y()) || std::isnan(q.z()) || std::isnan(q.w())) {
        valid_orientation = false;
        ROS_DEBUG_THROTTLE(1.0, "IMU四元数无效，使用角速度积分估计姿态");
    }
    
    // 获取当前时间和时间差
    ros::Time current_time = latest_imu_.header.stamp;
    double dt = (current_time - last_orientation_time).toSec();
    
    // 确保时间差是合理的
    if (dt <= 0 || dt > 1.0) {
        dt = 0.01; // 设置一个合理的默认值
        ROS_WARN_THROTTLE(1.0, "IMU时间间隔异常(%.4f秒)，使用默认值", dt);
    }
    
    // 根据IMU数据更新姿态
    if (!valid_orientation) {
        // 如果四元数无效，使用角速度积分估计姿态
        Eigen::Vector3f gyro(
            latest_imu_.angular_velocity.x,
            latest_imu_.angular_velocity.y,
            latest_imu_.angular_velocity.z
        );
        
        // 检查角速度是否有效
        if (std::isnan(gyro.x()) || std::isnan(gyro.y()) || std::isnan(gyro.z()) ||
            std::isinf(gyro.x()) || std::isinf(gyro.y()) || std::isinf(gyro.z())) {
            ROS_WARN_THROTTLE(1.0, "IMU角速度无效，跳过姿态更新");
        } else {
            // 创建从角速度导出的四元数增量
            float angle = gyro.norm() * dt;
            if (angle > 0.0001f) { // 避免除以接近零的值
                Eigen::Vector3f axis = gyro.normalized();
                Eigen::Quaternionf dq(Eigen::AngleAxisf(angle, axis));
                
                // 更新估计的姿态
                estimated_orientation = estimated_orientation * dq;
                
                // 归一化四元数以防止数值累积误差
                estimated_orientation.normalize();
            }
        }
        
        // 使用估计的姿态
        q = tf::Quaternion(
            estimated_orientation.x(),
            estimated_orientation.y(),
            estimated_orientation.z(),
            estimated_orientation.w()
        );
    } else {
        // 如果IMU提供了有效的四元数，更新估计的姿态
        estimated_orientation = Eigen::Quaternionf(q.w(), q.x(), q.y(), q.z());
    }
    
    // 更新上次处理时间
    last_orientation_time = current_time;
    
    // 检查四元数是否归一化
    double norm = std::sqrt(q.x()*q.x() + q.y()*q.y() + q.z()*q.z() + q.w()*q.w());
    if (std::abs(norm - 1.0) > 1e-3) {
        if (norm > 1e-10) { // 防止除以接近零的值
            q = tf::Quaternion(q.x()/norm, q.y()/norm, q.z()/norm, q.w()/norm);
        } else {
            q = tf::Quaternion(0, 0, 0, 1);
        }
    }
    
    // 转换为Eigen四元数
    Eigen::Quaternionf current_orientation = Eigen::Quaternionf(q.w(), q.x(), q.y(), q.z());
    
    // 初始化参考方向（如果是第一次）
    if (!orientation_initialized) {
        // 在第一次有效的姿态数据时，初始化参考方向
        init_orientation_ = current_orientation;
        orientation_initialized = true;
        ROS_INFO("姿态参考方向已初始化");
    }
    
    // 计算相对于初始方向的旋转
    Eigen::Quaternionf corrected_orientation = init_orientation_.inverse() * current_orientation;
    
    // 构建旋转矩阵
    Eigen::Matrix3f rotation_matrix = corrected_orientation.toRotationMatrix();
    
    // 积分位移（使用更稳定的方法）
    static Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
    static Eigen::Vector3f position = Eigen::Vector3f::Zero();
    static ros::Time last_pos_time = ros::Time::now();
    
    // 重新计算时间差（用于位置积分）
    dt = (current_time - last_pos_time).toSec();
    
    // 确保时间差是合理的
    if (dt > 0 && dt < 1.0) {
        // 获取加速度数据
        Eigen::Vector3f accel(
            latest_imu_.linear_acceleration.x,
            latest_imu_.linear_acceleration.y,
            latest_imu_.linear_acceleration.z
        );
        
        // 检查加速度是否有效
        if (std::isnan(accel.x()) || std::isnan(accel.y()) || std::isnan(accel.z()) ||
            std::isinf(accel.x()) || std::isinf(accel.y()) || std::isinf(accel.z())) {
            ROS_WARN_THROTTLE(1.0, "IMU加速度无效，跳过位置更新");
        } else {
            // 将加速度从IMU坐标系转换到世界坐标系，并减去重力
            Eigen::Vector3f global_acc = rotation_matrix * accel - Eigen::Vector3f(0, 0, 9.81);
            
            // 应用简单的低通滤波器减少噪声（可选）
            static Eigen::Vector3f filtered_acc = Eigen::Vector3f::Zero();
            float alpha = 0.2f; // 滤波器系数，较小的值滤波效果更强
            filtered_acc = alpha * global_acc + (1-alpha) * filtered_acc;
            
            // 使用滤波后的加速度积分获取速度和位置
            velocity += filtered_acc * dt;
            
            // 简单的速度阻尼（模拟摩擦力，防止速度无限增长）
            float damping = 0.98f;
            velocity *= damping;
            
            // 积分速度获取位置
            position += velocity * dt;
        }
        
        last_pos_time = current_time;
    } else if (dt <= 0) {
        ROS_WARN_THROTTLE(5.0, "IMU时间戳无效，跳过位置更新");
    } else if (dt >= 1.0) {
        ROS_WARN_THROTTLE(5.0, "IMU时间间隔过大(%.2f秒)，重置速度", dt);
        velocity = Eigen::Vector3f::Zero();
        last_pos_time = current_time;
    }
    
    // 构建完整的变换矩阵
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3,3>(0,0) = rotation_matrix;
    transform.block<3,1>(0,3) = position;
    
    // 检查所有值是否有效
    if (!std::isnan(position.x()) && !std::isnan(position.y()) && !std::isnan(position.z()) &&
        !std::isnan(corrected_orientation.x()) && !std::isnan(corrected_orientation.y()) && 
        !std::isnan(corrected_orientation.z()) && !std::isnan(corrected_orientation.w())) {
        
        // 广播TF变换
        tf::Transform tf_transform;
        tf_transform.setOrigin(tf::Vector3(position.x(), position.y(), position.z()));
        tf_transform.setRotation(tf::Quaternion(
            corrected_orientation.x(),
            corrected_orientation.y(),
            corrected_orientation.z(),
            corrected_orientation.w()
        ));
        tf_broadcaster_.sendTransform(tf::StampedTransform(
            tf_transform, current_time, global_frame_id_, lidar_frame_id_
        ));
    } else {
        ROS_ERROR_THROTTLE(1.0, "无法发布变换，包含NaN值");
        // 返回单位变换，避免传播错误
        return Eigen::Matrix4f::Identity();
    }
    
    return transform;
}

void GlobalPointcloudMapping::cropPointsWithinRadius(
    const PointCloudT::Ptr& cloud_in, 
    PointCloudT::Ptr& cloud_out, 
    float radius
) {
    // 手动实现球形裁剪
    cloud_out->clear();
    const float radius_squared = radius * radius;
    
    // 遍历所有点
    for (const auto& point : cloud_in->points) {
        // 计算点到原点的平方距离
        const float distance_squared = 
            point.x * point.x + 
            point.y * point.y + 
            point.z * point.z;
        
        // 如果点在球体内部，则保留
        if (distance_squared <= radius_squared) {
            cloud_out->points.push_back(point);
        }
    }
    
    // 设置点云属性
    cloud_out->width = cloud_out->points.size();
    cloud_out->height = 1;
    cloud_out->is_dense = cloud_in->is_dense;
}

void GlobalPointcloudMapping::rebuildGlobalMapFromVoxels() {
    // 清空全局点云
    global_map_.reset(new PointCloudT());
    
    // 对于每个网格，选择权重最高的点添加到全局点云
    for (const auto& voxel_pair : voxel_map_) {
        const VoxelData& voxel = voxel_pair.second;
        
        if (voxel.points.empty()) {
            continue;
        }
        
        // 找到权重最高的点
        size_t max_weight_idx = 0;
        float max_weight = voxel.weights[0];
        
        for (size_t i = 1; i < voxel.weights.size(); ++i) {
            if (voxel.weights[i] > max_weight) {
                max_weight = voxel.weights[i];
                max_weight_idx = i;
            }
        }
        
        // 将权重最高的点添加到全局点云
        global_map_->push_back(voxel.points[max_weight_idx]);
    }
    
    // 设置点云属性
    global_map_->width = global_map_->points.size();
    global_map_->height = 1;
    global_map_->is_dense = false;
}

void GlobalPointcloudMapping::publishGlobalMap() {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    if (global_map_->empty()) {
        return;
    }
    
    // 转换为ROS消息
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*global_map_, cloud_msg);
    cloud_msg.header.frame_id = global_frame_id_;
    cloud_msg.header.stamp = ros::Time::now();
    
    // 发布全局点云地图
    global_map_pub_.publish(cloud_msg);
}

void GlobalPointcloudMapping::cleanOldVoxels(const ros::Time& current_time, double clean_threshold) {
    // 清理长时间未更新的网格
    std::vector<Eigen::Vector3i> keys_to_remove;
    
    for (const auto& voxel_pair : voxel_map_) {
        double time_diff = (current_time - voxel_pair.second.latest_timestamp).toSec();
        
        // 只有超过时间阈值的网格才会被清理
        if (time_diff > clean_threshold) {
            keys_to_remove.push_back(voxel_pair.first);
        }
    }
    
    // 删除过期网格
    for (const auto& key : keys_to_remove) {
        voxel_map_.erase(key);
    }
    
    if (!keys_to_remove.empty()) {
        ROS_INFO("清理了 %zu 个过期网格", keys_to_remove.size());
    }
}