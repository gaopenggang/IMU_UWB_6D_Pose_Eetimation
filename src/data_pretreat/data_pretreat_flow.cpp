#include "imu_uwb/data_pretreat/data_pretreat_flow.hpp"

namespace IMU_UWB {
DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh) {
    // subscriber
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/imu/data_xsens", 1000000);
    uwb_sub_ptr_ = std::make_shared<UWBSubscriber>(nh, "/nlink_linktrack_nodeframe2", 1000000);
    // publisher
    imu_pub_ptr_=std::make_shared<IMUPublisher>(nh, "/synced_imu",  "/imu_link", 100);
    uwb_pub_ptr_=std::make_shared<UWBPublisher>(nh, "/synced_uwb", "/uwb_link", 100);

}

bool DataPretreatFlow::Run() {
    if (!ReadData())
        return false;

    while(HasData()) {
        if (!ValidData())
            continue;

        PublishData();
    }

    return true;
}

bool DataPretreatFlow::ReadData() {
    uwb_sub_ptr_->ParseData(uwb_data_buff_);

    static std::deque<IMUData> unsynced_imu_;

    imu_sub_ptr_->ParseData(unsynced_imu_);

    if (uwb_data_buff_.size() == 0)
        return false;

    double uwb_time = uwb_data_buff_.front().time;
    bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, uwb_time);

    static bool sensor_inited = false;
    if (!sensor_inited) {
        if (!valid_imu ) {
            //std::cout.precision(32);
            //std::cout<<"unsynced_imu_.front().time "<<unsynced_imu_.front().time<<std::endl;
            //std::cout<<"uwb_data_buff_.front().time "<<uwb_data_buff_.front().time<<std::endl;

            uwb_data_buff_.pop_front();

            return false;
        }
        sensor_inited = true;
   }
    return true;
}

bool DataPretreatFlow::HasData() {
    if (uwb_data_buff_.size() == 0)
        return false;
    if (imu_data_buff_.size() == 0)
        return false;

    return true;
}
// conform that data have the same time stamp
bool DataPretreatFlow::ValidData() {
    current_uwb_data_ = uwb_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();

    double diff_imu_time = current_uwb_data_.time - current_imu_data_.time;

    if (diff_imu_time < -0.05 ) {
        uwb_data_buff_.pop_front();
        return false;
    }

    if (diff_imu_time > 0.05) {
        imu_data_buff_.pop_front();
        return false;
    }


    uwb_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    return true;
}


bool DataPretreatFlow::PublishData() {
    uwb_pub_ptr_->Publish(current_uwb_data_);
    imu_pub_ptr_->Publish(current_imu_data_); //////

    return true;
}
}