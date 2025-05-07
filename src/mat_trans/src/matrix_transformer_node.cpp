#include "matrix_transformer.h"








int main(int argc, char** argv) {
    ros::init(argc, argv, "matrix_transformer_node");
    ros::NodeHandle nh;


    
    MatrixTransformer transformer;

    // Set 1Hz publishing rate
    ros::Rate rate(1); // 1 Hz
    while (ros::ok()) {
        transformer.publishPointCloud();
        transformer.testpublishPointCloud();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}