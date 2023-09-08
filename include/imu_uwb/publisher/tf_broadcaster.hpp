
#ifndef IMU_UWB_PUBLISHER_TF_BROADCASTER_HPP_
#define IMU_UWB_PUBLISHER_TF_BROADCASTER_HPP_

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

namespace IMU_UWB {
class TFBroadCaster {
  public:
    TFBroadCaster(std::string frame_id, std::string child_frame_id);
    TFBroadCaster() = default;
    void SendTransform(Eigen::Matrix4d pose, double time);
  protected:
    tf::StampedTransform transform_;
    tf::TransformBroadcaster broadcaster_;
};
}
#endif