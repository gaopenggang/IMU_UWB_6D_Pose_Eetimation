#ifndef IMU_UWB_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_
#define IMU_UWB_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_

#include <ros/ros.h>
// subscriber
#include "imu_uwb/subscriber/imu_subscriber.hpp"
#include "imu_uwb/subscriber/uwb_subscriber.hpp"
//#include "imu_uwb/tf_listener/tf_listener.hpp"
// publisher
#include "imu_uwb/publisher/imu_publisher.hpp"
#include "imu_uwb/publisher/uwb_publisher.hpp"
//sensor data
#include "imu_uwb/sensor_data/imu_data.hpp"
#include "imu_uwb/sensor_data/uwb_data.hpp"


namespace IMU_UWB {
class DataPretreatFlow {
  public:
    DataPretreatFlow(ros::NodeHandle& nh);

    bool Run();

  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool PublishData();

  private:
    // subscriber
    std::shared_ptr<UWBSubscriber> uwb_sub_ptr_;
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    //std::shared_ptr<TFListener> uwb_to_imu_ptr_;
    // publisher
    std::shared_ptr<IMUPublisher> imu_pub_ptr_;
    std::shared_ptr<UWBPublisher> uwb_pub_ptr_;

    Eigen::Matrix4f uwb_to_imu_ = Eigen::Matrix4f::Identity();

    std::deque<IMUData> imu_data_buff_;
    std::deque<UWBData> uwb_data_buff_;

    IMUData current_imu_data_;
    UWBData current_uwb_data_;

};
}

#endif