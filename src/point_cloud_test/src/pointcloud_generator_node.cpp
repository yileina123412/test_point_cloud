#include "pointcloud_generator.h"








int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_generator");
    PointCloudGenerator generator;
    ros::Rate rate(10); // 10 Hz
    while (ros::ok()) {
        generator.generateAndPublish();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}



