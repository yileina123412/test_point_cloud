#include "obstacle_detector.h"

int main(int argc, char** argv) {
    // 设置中文支持
    setlocale(LC_ALL, "");
    
    // 初始化ROS节点
    ros::init(argc, argv, "obstacle_detector_node");
    ros::NodeHandle nh;
    
    ROS_INFO("Starting Obstacle Detector Node...");
    
    try {
        // 创建障碍物检测器
        ObstacleDetector detector;
        
        // 设置处理频率
        double process_rate = 10.0;  // 默认10Hz
        ros::param::param<double>("~process_rate", process_rate, 10.0);
        
        ROS_INFO("Obstacle detector running at %.1f Hz", process_rate);
        
        // 运行主循环
        ros::spin();
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in obstacle detector: %s", e.what());
        return -1;
    }
    
    ROS_INFO("Obstacle Detector Node shutting down...");
    return 0;
}

