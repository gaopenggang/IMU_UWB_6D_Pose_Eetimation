
#ifndef IMU_UWB_SUBSCRIBER_UWB_SUBSCRIBER_HPP_
#define IMU_UWB_SUBSCRIBER_UWB_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "nlink_parser/LinktrackNode2.h"
#include "nlink_parser/LinktrackNodeframe2.h"
#include "imu_uwb/sensor_data/uwb_data.hpp"

namespace IMU_UWB {
class UWBSubscriber {
  public:
    UWBSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    UWBSubscriber() = default;
    void ParseData(std::deque<UWBData>& deque_uwb_data);

  private:
    void msg_callback(const nlink_parser::LinktrackNodeframe2ConstPtr& uwb_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<UWBData> new_uwb_data_;

    std::mutex buff_mutex_;
};
}
#endif