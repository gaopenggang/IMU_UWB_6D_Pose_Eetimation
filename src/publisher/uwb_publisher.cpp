#include "imu_uwb/publisher/uwb_publisher.hpp"

#include <Eigen/Dense>

namespace IMU_UWB {
UWBPublisher::UWBPublisher(ros::NodeHandle& nh, 
                                     std::string topic_name, 
                                     std::string frame_id,
                                     int buff_size)
    :nh_(nh), frame_id_(frame_id) {

    publisher_ = nh_.advertise<nlink_parser::LinktrackNodeframe2>(topic_name, buff_size);
}

void UWBPublisher::Publish(UWBData uwb_data) {
    nlink_parser::LinktrackNodeframe2 uwb_data_;
    nlink_parser::LinktrackNode2 uwb_node;

    ros::Time ros_time((float)uwb_data.time);
    uwb_data_.header.stamp = ros_time;
    uwb_data_.header.frame_id = frame_id_;
    uwb_data_.id=uwb_data.id;
    int nodes_size=uwb_data.connected_nodes.size();
    for(int i=0;i<nodes_size;i++){
        uwb_node.role = uwb_data.connected_nodes[i].role;
        uwb_node.id = uwb_data.connected_nodes[i].id;
        uwb_node.dis= uwb_data.connected_nodes[i].dis;
        uwb_node.fp_rssi = uwb_data.connected_nodes[i].fp_rssi;
        uwb_node.rx_rssi = uwb_data.connected_nodes[i].rx_rssi;

        uwb_data_.nodes.push_back(uwb_node);

    }

    publisher_.publish(uwb_data_);
}


}