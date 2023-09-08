#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include "imu_uwb/trajectory/trajectory_fitting.hpp"

using namespace IMU_UWB;

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "ground_truth_node");
    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("ground_truth",1, true);
    nav_msgs::Path path;
    path.header.stamp=ros::Time::now();
    path.header.frame_id="map";

    Trajectory trajectory;
    trajectory.ReadTrajectory();

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        int traj_size=trajectory.trajectory_poses.size();
        for(int i=0;i<traj_size;i++)
        {
            geometry_msgs::PoseStamped traj_pose_stamped;
            traj_pose_stamped.pose.position.x = trajectory.trajectory_poses[i].t(0);
            traj_pose_stamped.pose.position.y = trajectory.trajectory_poses[i].t(1);
            traj_pose_stamped.pose.position.z = trajectory.trajectory_poses[i].t(2);

            traj_pose_stamped.pose.orientation.x = trajectory.trajectory_poses[i].q.x();
            traj_pose_stamped.pose.orientation.y = trajectory.trajectory_poses[i].q.y();
            traj_pose_stamped.pose.orientation.z = trajectory.trajectory_poses[i].q.z();
            traj_pose_stamped.pose.orientation.w = trajectory.trajectory_poses[i].q.w();

            traj_pose_stamped.header.stamp=ros::Time::now();
            traj_pose_stamped.header.frame_id="map";
            path.poses.push_back(traj_pose_stamped);
        }
        path_pub.publish(path);
        rate.sleep();
    }

    return 0;
}