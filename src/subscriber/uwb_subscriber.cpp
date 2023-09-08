#include "imu_uwb/subscriber/uwb_subscriber.hpp"

namespace IMU_UWB{
UWBSubscriber::UWBSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size) 
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &UWBSubscriber::msg_callback, this);
}

void UWBSubscriber::msg_callback(const nlink_parser::LinktrackNodeframe2ConstPtr& uwb_msg_ptr) {
    buff_mutex_.lock();
        UWBData uwb_data;
        //std::cout.precision(32);
        //std::cout<<"uwb_msg_ptr->header.stamp.toSec()"<<uwb_msg_ptr->header.stamp.toSec()<<std::endl;
        uwb_data.time = uwb_msg_ptr->header.stamp.toSec();
        uwb_data.id=uwb_msg_ptr->id;
        for(auto data : uwb_msg_ptr->nodes){
            if(data.rx_rssi-data.fp_rssi > 6.0) //rx_rssi-fp_rss <6 : bad data (NLOP)
                continue;
            uwb_data.connected_uwb_node.id = data.id;
            uwb_data.connected_uwb_node.dis= data.dis;
            uwb_data.connected_uwb_node.role= data.role;
            uwb_data.connected_uwb_node.fp_rssi= data.fp_rssi;
            uwb_data.connected_uwb_node.rx_rssi= data.rx_rssi;

            uwb_data.connected_nodes.push_back(uwb_data.connected_uwb_node);
        }
        new_uwb_data_.push_back(uwb_data);
    buff_mutex_.unlock();
    }

void UWBSubscriber::ParseData(std::deque<UWBData>& uwb_data_buff) {
    buff_mutex_.lock();
    if (new_uwb_data_.size() > 0) {
        uwb_data_buff.insert(uwb_data_buff.end(), new_uwb_data_.begin(), new_uwb_data_.end());
        new_uwb_data_.clear();
    }
    buff_mutex_.unlock();
}
}
