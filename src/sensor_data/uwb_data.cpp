#include "imu_uwb/sensor_data/uwb_data.hpp"
namespace IMU_UWB {

void UWBData::TransformCoordinate(Eigen::Matrix4f transform_matrix) {
    Eigen::Matrix4d matrix = transform_matrix.cast<double>();
    Eigen::Matrix3d t_R = matrix.block<3,3>(0,0);
    Eigen::Vector3d r(matrix(0,3), matrix(1,3), matrix(2,3));


}
}