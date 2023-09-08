//
// Created by xc on 2021/3/8.
//

#include "imu_uwb/utility/keyFrame.h"

namespace IMU_UWB{
    KeyFrame::KeyFrame() {
        state_.clear();
    }

    KeyFrame::~KeyFrame() {}

    pcl::PointCloud<pcl::PointXYZI> KeyFrame::associatePointCloudToMap(){
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformedPointcloud (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointXYZI point_transformed;
        cloud += surf_cloud_;
        cloud += corner_cloud_;
        size_t points_n = cloud.points.size();
        for(size_t i=0;i<points_n;i++){
            Eigen::Vector3d point_curr(cloud.points.at(i).x,cloud.points.at(i).y,cloud.points.at(i).z);
            Eigen::Vector3d point_w = state_.quaternion_ * point_curr + state_.position_;
            point_transformed.x = point_w.x();
            point_transformed.y = point_w.y();
            point_transformed.z = point_w.z();
            transformedPointcloud->points.push_back(point_transformed);

        }
        return *transformedPointcloud;
    }

    pcl::PointCloud<pcl::PointXYZI> KeyFrame::getTransformedCloud() {
        pcl::PointCloud<pcl::PointXYZI> cloud,transformedcloud;
        cloud += surf_cloud_;
        cloud += corner_cloud_;
        pcl::transformPointCloud(cloud,transformedcloud,state_.getTransformation2());
        return transformedcloud;
    }

    pcl::PointCloud<pcl::PointXYZI> KeyFrame::getTransformedSurfCloud() {
        pcl::PointCloud<pcl::PointXYZI> transformedcloud;
        pcl::transformPointCloud(surf_cloud_,transformedcloud,state_.getTransformation2());
        return transformedcloud;
    }

    pcl::PointCloud<pcl::PointXYZI> KeyFrame::getTransformedCornerCloud() {
        pcl::PointCloud<pcl::PointXYZI> transformedcloud;
        pcl::transformPointCloud(corner_cloud_,transformedcloud,state_.getTransformation2());
        return transformedcloud;
    }



}
