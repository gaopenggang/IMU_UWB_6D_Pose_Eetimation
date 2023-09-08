#include "imu_uwb/publisher/imu_publisher.hpp"

#include <Eigen/Dense>

namespace IMU_UWB {
IMUPublisher::IMUPublisher(ros::NodeHandle& nh,
                                     std::string topic_name,
                                     std::string frame_id,
                                     int buff_size)
    :nh_(nh), frame_id_(frame_id) {

    publisher_ = nh_.advertise<sensor_msgs::Imu>(topic_name, buff_size);
}

void IMUPublisher::Publish(IMUData imu_data) {
    sensor_msgs::Imu imu;

    ros::Time ros_time((float)imu_data.time);
    imu.header.stamp = ros_time;
    imu.header.frame_id = frame_id_;

    imu.orientation.x = imu_data.orientation.x;
    imu.orientation.y = imu_data.orientation.y;
    imu.orientation.z = imu_data.orientation.z;
    imu.orientation.w = imu_data.orientation.w;

    imu.angular_velocity.x = imu_data.angular_velocity.x;
    imu.angular_velocity.y = imu_data.angular_velocity.y;
    imu.angular_velocity.z = imu_data.angular_velocity.z;

    imu.linear_acceleration.x=imu_data.linear_acceleration.x;
    imu.linear_acceleration.y=imu_data.linear_acceleration.y;
    imu.linear_acceleration.z=imu_data.linear_acceleration.z;


    publisher_.publish(imu);
}

}