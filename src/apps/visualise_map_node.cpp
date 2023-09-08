//
// Created by on 2021/6/18.
//
#include <ros/ros.h>
#include "ros/subscriber.h"
#include "ros/publisher.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include "imu_uwb/utility/keyFrame.h"

using namespace IMU_UWB;

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "visualise_map_node");
    ros::NodeHandle nh;
    std::string data_dir_path_;
    nh.getParam("/data_dir_path", data_dir_path_);
    ros::Publisher pub_global_map_ = nh.advertise<sensor_msgs::PointCloud2>("/globalmap",2);

    pcl::PointCloud<pcl::PointXYZI> global_map_;
    std::string map_dir_path = data_dir_path_ + "/map";
    std::string pg_path = data_dir_path_ + "/pose_graph/pose_graph.txt";
    std::FILE* pg_ifile = std::fopen(pg_path.c_str(),"r");
    bool success_read_map;
    if (pg_ifile == NULL) {
        std::cout << "Error ! Open [ " << pg_path << " ] failed ..." << std::endl;
        success_read_map=false;
    }
    else{
        int i;
        KeyFrame tmp_kf;
        pcl::PointCloud<pcl::PointXYZI> surf_map;
        double t, px, py, pz, qx, qy, qz, qw;
        while(std::fscanf(pg_ifile,"%d %lf %lf %lf %lf %lf %lf %lf %lf",&i, &t,&px,&py,&pz,&qx,&qy,&qz,&qw)!=EOF){
            //std::cout << "i " << i <<std::endl;
            tmp_kf.state_.position2_.x() = px;
            tmp_kf.state_.position2_.y() = py;
            tmp_kf.state_.position2_.z() = pz;
            tmp_kf.state_.quaternion2_.x() = qx;
            tmp_kf.state_.quaternion2_.y() = qy;
            tmp_kf.state_.quaternion2_.z() = qz;
            tmp_kf.state_.quaternion2_.w() = qw;
            pcl::io::loadPCDFile(map_dir_path + "/" + std::to_string(i) + "_surf_cloud.pcd",tmp_kf.surf_cloud_ );
            pcl::io::loadPCDFile(map_dir_path + "/" + std::to_string(i) + "_corner_cloud.pcd",tmp_kf.corner_cloud_ );
            global_map_ += tmp_kf.getTransformedCloud();
            surf_map += tmp_kf.surf_cloud_;
        }
        std::fclose(pg_ifile);
        ROS_ERROR_STREAM("load map done ...  with size : " << global_map_.points.size() << " surf : " << surf_map.points.size()<<"\n");
        success_read_map=true;
    }

    ros::Rate rate(1);
    while (ros::ok()) {
        ros::spinOnce();
        if(success_read_map)
        {
            sensor_msgs::PointCloud2 map_msg;
            pcl::toROSMsg(global_map_,map_msg);
            map_msg.header.frame_id = "map";
            map_msg.header.stamp = ros::Time::now();
            pub_global_map_.publish(map_msg);
        }
        rate.sleep();
    }

    return 0;
}
