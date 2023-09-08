#include <ros/ros.h>

#include "imu_uwb/fusion/imu_uwb_fusion_flow.hpp"

using namespace IMU_UWB;

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "imu_uwb_location_node");
    ros::NodeHandle nh;

    std::shared_ptr<LocalizationFlow> localization_flow_ptr = std::make_shared<LocalizationFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        localization_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}