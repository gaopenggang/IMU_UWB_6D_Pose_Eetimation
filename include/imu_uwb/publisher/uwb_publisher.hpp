#ifndef IMU_UWB_PUBLISHER_UWB_PUBLISHER_HPP_
#define IMU_UWB_PUBLISHER_UWB_PUBLISHER_HPP_

#include "sensor_msgs/Imu.h"
#include "imu_uwb/sensor_data/uwb_data.hpp"
#include <ros/ros.h>
namespace IMU_UWB {
    class UWBPublisher {
    public:
        UWBPublisher(ros::NodeHandle& nh,
                     std::string topic_name,
                     std::string frame_id,
                     int buff_size);
        UWBPublisher() = default;

        void Publish(UWBData uwb_data);

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_;
    };
}
#endif