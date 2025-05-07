#include "global_point_mapping/global_pointcloud_mapping.h"
#include <ros/ros.h>

int main(int argc, char** argv) {

    setlocale(LC_ALL, "");
    // 初始化ROS节点
    ros::init(argc, argv, "global_pointcloud_mapping_node");
    ros::NodeHandle nh("~");
    
    // 创建全局点云构图对象
    GlobalPointcloudMapping mapper(nh);
    
    // 初始化
    if (!mapper.init()) {
        ROS_ERROR("全局点云构图系统初始化失败!");
        return -1;
    }
    
    // 使用ROS异步机制处理回调
    ros::AsyncSpinner spinner(1); // 使用1个线程
    spinner.start();
    
    // 运行主循环
    mapper.run();
    
    // 停止ROS异步线程（实际上不会执行到这里，因为run()是阻塞的）
    spinner.stop();
    
    return 0;
}