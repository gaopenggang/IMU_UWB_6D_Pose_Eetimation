#ifndef IMU_UWB_TRAJECTORY_HPP_
#define IMU_UWB_TRAJECTORY_HPP_
#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include<Eigen/Core>

namespace IMU_UWB{
    class Trajectory{
        public:
        struct TrajectoryPose
        {
            Eigen::Quaterniond q;
            Eigen::Vector3d t;
            double uwb_time;
        };

        TrajectoryPose trajectory_pose;
        std::vector<TrajectoryPose> trajectory_poses;
        std::string data_dir_path_="/home/xc/2021-0916/result_data";
        Eigen::Matrix<double,1,10> observation;

        void ReadTrajectory();
        Eigen::Matrix<double, 1, 10> CalculateObservation(int id);
        int CalculateNearestPoseID(Eigen::Vector3d position);

    };

}
#endif
