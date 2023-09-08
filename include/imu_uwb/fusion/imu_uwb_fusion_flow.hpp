#ifndef IMU_UWB_LOCALIZATION_FLOW_HPP_
#define IMU_UWB_LOCALIZATION_FLOW_HPP_
#include <unistd.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <yaml-cpp/yaml.h>
//subscriber
#include "imu_uwb/subscriber/imu_subscriber.hpp"
#include "imu_uwb/subscriber/uwb_subscriber.hpp"
#include "imu_uwb/sensor_data/imu_data.hpp"
#include "imu_uwb/sensor_data/uwb_data.hpp"
//publisher
#include "imu_uwb/publisher/odometry_publisher.hpp"
#include "imu_uwb/publisher/tf_broadcaster.hpp"
//init
#include "imu_uwb/fusion/init.hpp"
#include "imu_uwb/trajectory/trajectory_fitting.hpp"

namespace IMU_UWB
{
class LocalizationFlow
  {
  public:
    struct State
    {
      Eigen::Vector3d p;
      Eigen::Vector3d v;
      Eigen::Quaterniond q;
      Eigen::Vector3d ba;
      Eigen::Vector3d bw;
    };
    struct ErrorState
    {
      Eigen::Matrix<double, 15, 1> x;
      Eigen::Matrix<double, 15, 15> p;
    };
    LocalizationFlow(ros::NodeHandle &nh);
    // init part
    InitPosition init_position_class;
    Eigen::Vector3d first_ekf_init_position;
    Eigen::Vector3d second_ekf_init_position;
    bool init_flag=false;
    Trajectory trajectory_;
    ros::NodeHandle nh_;
    visualization_msgs::Marker marker;
    ros::Publisher marker_pub = nh_.advertise<visualization_msgs::Marker>("traj_ref", 100);
    visualization_msgs::Marker uwb_station_marker;
    ros::Publisher uwb_station_marker_pub = nh_.advertise<visualization_msgs::Marker>("live_uwb_station", 100);
    bool Run();

  private:
    bool ReadData();
    bool InitEKF();
    bool SyncData();
    bool Filter();
    bool Predict();
    bool Correct();
    bool PublishData();

  private:
    // subscriber
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<UWBSubscriber> uwb_sub_ptr_;
   //std::shared_ptr<TFListener> uwb_to_imu_ptr_;
    // publisher
    std::shared_ptr<OdometryPublisher> odom_pub_ptr_;
    std::shared_ptr<TFBroadCaster> tf_pub_ptr_;

    //Eigen::Matrix4d uwb_to_imu_ = Eigen::Matrix4d::Identity();

    std::deque<IMUData> imu_data_buff_;
    std::deque<UWBData> uwb_data_buff_;

    //IMUData current_imu_data_;
    UWBData current_uwb_data_;
    std::deque<IMUData> current_imu_data_;
    double sync_time;

    State state_;
    ErrorState error_state_;
    double gyro_noise_;
    double acc_noise_;
    double gyro_bias_noise_;
    double acc_bias_noise_;
    double uwb_noise_;
    double trajectory_noise_;
    double non_hol_noise_z_;

    double room_0_x_;
    double room_0_y_;
    double corridor_0_x_0_;
    double corridor_0_x_1_;
    double corridor_0_y_0_;
    double corridor_0_y_1_;
    double corridor_1_x_0_;
    double corridor_1_x_1_;
    double corridor_1_y_0_;
    double corridor_1_y_1_;
    double corridor_2_x_0_;
    double corridor_2_x_1_;
    double corridor_2_y_0_;
    double corridor_2_y_1_;
    double room_1_x_;
    double room_1_y_;
    double glass_room_x_0_;
    double glass_room_x_1_;
    double glass_room_y_0_;
    double glass_room_y_1_;
    double glass_room_0_x_0_;
    double glass_room_0_x_1_;
    double glass_room_0_y_0_;
    double glass_room_0_y_1_;



};
  }
  #endif