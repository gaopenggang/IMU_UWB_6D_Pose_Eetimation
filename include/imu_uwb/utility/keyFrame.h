//
// Created by on 2021/3/8.
//

#ifndef ALOAM_VELODYNE_KEYFRAME_H
#define ALOAM_VELODYNE_KEYFRAME_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <array>
#include <Eigen/Dense>
#include<Eigen/Core>
#include <string>

namespace IMU_UWB{
    class State{
    public:
        State(){
            clear();
        };
        ~State(){};
        void clear()
        {
            timestamp_ = 0.0;
            relative_pose_.setIdentity();
            absolut_pose_.setIdentity();
            position_.setZero();
            position2_.setZero();
            quaternion_.setIdentity();
            quaternion2_.setIdentity();
            linear_velocity_.setZero();
            angular_velocity_.setZero();
            bias_a_.setZero();
            bias_g_.setZero();
        };

        Eigen::Matrix4d getTransformation()
        {
            Eigen::Matrix4d mat4d;
            mat4d.setIdentity();
            mat4d.block<3, 3>(0, 0) = quaternion_.toRotationMatrix();
            mat4d.block<3, 1>(0, 3) = position_;
            return mat4d;
        };

        Eigen::Matrix4d getTransformation2()
        {
            Eigen::Matrix4d mat4d;
            mat4d.setIdentity();
            mat4d.block<3, 3>(0, 0) = quaternion2_.toRotationMatrix();
            mat4d.block<3, 1>(0, 3) = position2_;
            return mat4d;
        };

        void toXYZRPY(){
            Eigen::Vector3d euler = quaternion_.toRotationMatrix().eulerAngles(2,1,0);
            xyzrpy_[0] = position_.x();
            xyzrpy_[1] = position_.y();
            xyzrpy_[2] = position_.z();
            xyzrpy_[3] = euler.x();
            xyzrpy_[4] = euler.y();
            xyzrpy_[5] = euler.z();
        }

        void update(Eigen::Matrix4d& correction_matx){
            Eigen::Matrix4d origin_matx = getTransformation();
            Eigen::Matrix4d updated_matx = origin_matx * correction_matx;
            quaternion_ = Eigen::Quaterniond(updated_matx.block<3,3>(0,0).matrix());
            position_ = updated_matx.block<3,1>(0,3);
        }


    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double timestamp_;
        Eigen::Vector3d position_;
        Eigen::Vector3d position2_;
        Eigen::Vector3d linear_velocity_;
        Eigen::Vector3d angular_velocity_; // in body-frame, being transfromed to {world} is complicated and meaningless.
        Eigen::Vector3d bias_a_;
        Eigen::Vector3d bias_g_;
        Eigen::Quaterniond quaternion_;
        Eigen::Quaterniond quaternion2_;
        std::array<double ,6> xyzrpy_;
        Eigen::Matrix4d relative_pose_;
        Eigen::Matrix4d absolut_pose_;  // raw data from aloam odom
    };


    class KeyFrame{
    public:
        KeyFrame();
        ~KeyFrame();
        pcl::PointCloud<pcl::PointXYZI> associatePointCloudToMap();
        pcl::PointCloud<pcl::PointXYZI> getTransformedCloud();
        pcl::PointCloud<pcl::PointXYZI> getTransformedSurfCloud();
        pcl::PointCloud<pcl::PointXYZI> getTransformedCornerCloud();

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        State state_;
        pcl::PointCloud<pcl::PointXYZI> surf_cloud_;
        pcl::PointCloud<pcl::PointXYZI> corner_cloud_;


    };
}

#endif //ALOAM_VELODYNE_KEYFRAME_H