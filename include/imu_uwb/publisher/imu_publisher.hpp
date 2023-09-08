#ifndef IMU_UWB_PUBLISHER_IMU_PUBLISHER_HPP_
#define IMU_UWB_PUBLISHER_IMU_PUBLISHER_HPP_

#include "sensor_msgs/Imu.h"
#include "imu_uwb/sensor_data/imu_data.hpp"
#include <ros/ros.h>
namespace IMU_UWB {
    class IMUPublisher {
    public:
        IMUPublisher(ros::NodeHandle& nh,
                     std::string topic_name,
                              std::string frame_id,
                                     int buff_size);
        IMUPublisher() = default;

        void Publish(IMUData imu_data);

        bool HasSubscribers();

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_;
    };
}
#endif