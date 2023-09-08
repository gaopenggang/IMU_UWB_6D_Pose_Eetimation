#include "imu_uwb/fusion/imu_uwb_fusion_flow.hpp"

namespace IMU_UWB {
    LocalizationFlow::LocalizationFlow(ros::NodeHandle& nh) {
    // subscriber
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/imu/data_xsens", 1000000);
    uwb_sub_ptr_ = std::make_shared<UWBSubscriber>(nh, "/nlink_linktrack_nodeframe2", 1000000);

    // publisher
    odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/imu_uwb_localization", "/map", "/imu_link", 100);
    //tf_pub_ptr_ = std::make_shared<TFBroadCaster>("/map", "/imu_link");
    // traj observation part
    nh_=nh;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "shapes";
    marker.id = 0;
    marker.type =visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // uwb station part
    uwb_station_marker.header.frame_id = "/map";
    uwb_station_marker.header.stamp = ros::Time::now();
    uwb_station_marker.ns = "uwb_shapes";
    uwb_station_marker.id = 0;
    uwb_station_marker.type =visualization_msgs::Marker::SPHERE_LIST;//球序列;
    uwb_station_marker.action = visualization_msgs::Marker::ADD;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    uwb_station_marker.scale.x = 1.0;
    uwb_station_marker.scale.y = 1.0;
    uwb_station_marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    uwb_station_marker.color.r = 0.0f;
    uwb_station_marker.color.g = 1.0f;
    uwb_station_marker.color.b = 0.0f;
    uwb_station_marker.color.a = 1.0;

    std::string config_file_path = "/home/xc/catkin_ws/src/imu_uwb/config/kalman_filter_garage.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    gyro_noise_ = config_node["gyro_noise"].as<double>();
    acc_noise_ = config_node["acc_noise"].as<double>();
    gyro_bias_noise_ = config_node["gyro_bias_noise"].as<double>();
    acc_bias_noise_ = config_node["acc_bias_noise"].as<double>();
    uwb_noise_ = config_node["uwb_noise"].as<double>();
    //trajectory_noise_= config_node["trajectory_noise"].as<double>();
    non_hol_noise_z_=config_node["non_hol_noise_z"].as<double>();

   

    init_position_class.ReadUwbPosion();
    //trajectory_.ReadTrajectory();

}

bool LocalizationFlow::Run() {
   
    if (!ReadData())
        return false;

    if(SyncData()) {

        if (!InitEKF())
        {

        }
        else
        {
            Filter();
            PublishData();
        }
    }

    return true;
}

bool LocalizationFlow::ReadData() {
    static bool sensor_inited = false;
    uwb_sub_ptr_->ParseData(uwb_data_buff_);
    imu_sub_ptr_->ParseData(imu_data_buff_);


    if (uwb_data_buff_.empty() || imu_data_buff_.empty())
        return false;

    if (!sensor_inited)
    {
        while (!uwb_data_buff_.empty())
        {
            if (imu_data_buff_.front().time > uwb_data_buff_.front().time)
            {
                uwb_data_buff_.pop_front();
            }
            else
            {
                sensor_inited = true;
                break;
            }
        }
    }

    return sensor_inited;
}
bool LocalizationFlow::SyncData(){

    if (uwb_data_buff_.empty())
    {
        return false;
    }
    current_uwb_data_ = uwb_data_buff_.front();
    sync_time = uwb_data_buff_.front().time;

    if (imu_data_buff_.size() > 1)
    {
        if (imu_data_buff_.back().time < sync_time)
        {
            return false;
        }
        while (current_imu_data_.size() > 1)
        {
            current_imu_data_.pop_front();
        }
        while (imu_data_buff_.front().time < sync_time)
        {
        IMUData temp = imu_data_buff_.front();
        imu_data_buff_.pop_front();
        current_imu_data_.push_back(temp);
        }
        IMUData front_data = current_imu_data_.back();
        IMUData back_data = imu_data_buff_.at(0);
        IMUData synced_data;

        double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
        double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
        synced_data.time = sync_time;
        synced_data.linear_acceleration.x = front_data.linear_acceleration.x * front_scale + back_data.linear_acceleration.x * back_scale;
        synced_data.linear_acceleration.y = front_data.linear_acceleration.y * front_scale + back_data.linear_acceleration.y * back_scale;
        synced_data.linear_acceleration.z = front_data.linear_acceleration.z * front_scale + back_data.linear_acceleration.z * back_scale;
        synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
        synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
        synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;
        // 四元数插值有线性插值和球面插值，球面插值更准确，但是两个四元数差别不大是，二者精度相当
        // 由于是对相邻两时刻姿态插值，姿态差比较小，所以可以用线性插值
        synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
        synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
        synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
        synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;
        // 线性插值之后要归一化
        synced_data.orientation.Normlize();

        current_imu_data_.push_back(synced_data);
        uwb_data_buff_.pop_front();
         //std::cout << std::setprecision(12) << "current_imu_data_.time " << current_imu_data_.front().time
         //   << "  " << current_imu_data_.back().time << std::endl;
        return true;
    }
    else
    {
        return false;
    }
}

bool LocalizationFlow::InitEKF() {
    static bool ekf_inited = false;
    if(!ekf_inited)
    {
        double dis_0,dis_1,dis_2;
        int uwb_id_0, uwb_id_1,uwb_id_2;
        int linked_uwb_size=current_uwb_data_.connected_nodes.size();
        uwb_id_0=current_uwb_data_.connected_nodes[0].id;
        uwb_id_1=current_uwb_data_.connected_nodes[1].id;
        uwb_id_2=current_uwb_data_.connected_nodes[2].id;

        {
            if(!init_flag)
            {
                if(linked_uwb_size>=3)
                {
                    init_flag= true;
                    dis_0=current_uwb_data_.connected_nodes[0].dis;
                    dis_1=current_uwb_data_.connected_nodes[1].dis;
                    dis_2=current_uwb_data_.connected_nodes[2].dis;
                    uwb_id_0=current_uwb_data_.connected_nodes[0].id;
                    uwb_id_1=current_uwb_data_.connected_nodes[1].id;
                    uwb_id_2=current_uwb_data_.connected_nodes[2].id;

                    first_ekf_init_position=init_position_class.InitUwbTagPosition(dis_0,dis_1,dis_2,uwb_id_0,uwb_id_1,uwb_id_2);
                    sleep(3);

                }
                else
                {
                    std::cout<<"pls finish init in the room where enough uwb can be detected"<<std::endl;
                }
            }
            else
            {
                if(linked_uwb_size>=3)
                {
                    dis_0=current_uwb_data_.connected_nodes[0].dis;
                    dis_1=current_uwb_data_.connected_nodes[1].dis;
                    dis_2=current_uwb_data_.connected_nodes[2].dis;
                    uwb_id_0=current_uwb_data_.connected_nodes[0].id;
                    uwb_id_1=current_uwb_data_.connected_nodes[1].id;
                    uwb_id_2=current_uwb_data_.connected_nodes[2].id;
                    second_ekf_init_position=init_position_class.InitUwbTagPosition(dis_0,dis_1,dis_2,uwb_id_0,uwb_id_1,uwb_id_2);

                    float yaw = atan2((second_ekf_init_position[1] - first_ekf_init_position[1]), (second_ekf_init_position[0] - first_ekf_init_position[0]));
                    // eular to quaternion
                    Eigen::Vector3d eulerAngle(yaw,0,0);
                    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitX()));
                    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
                    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitZ()));

                    Eigen::Quaterniond quaternion;
                    quaternion=yawAngle*pitchAngle*rollAngle;

                    // temporaray
                    second_ekf_init_position[0]=0;
                    second_ekf_init_position[1]=0;
                    second_ekf_init_position[2]=0;
                    state_.p = second_ekf_init_position;
                    state_.q = quaternion;

                    state_.v<<0,0,0;
                    state_.bw = Eigen::Vector3d(0, 0, 0);
                    state_.ba = Eigen::Vector3d(0, 0, 0);
                    error_state_.x.setZero();
                    error_state_.p.setZero();
                    error_state_.p.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1e-2;
                    error_state_.p.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1e-3;
                    error_state_.p.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 3e-4;
                    error_state_.p.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 1e-4;
                    error_state_.p.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 1e-6;

                    std::cout<<"successful finished init"<<std::endl;
                    std::cout<<"init position= "<<second_ekf_init_position.transpose()<<std::endl;
                    std::cout<<"init orientation(yaw) = "<<yaw<<std::endl;
                    ekf_inited=true;
                }
                else
                {
                    std::cout<<"pls finish init in the room where enough uwb can be detected"<<std::endl;
                }
            }
        }
    }
    return ekf_inited;
}

bool LocalizationFlow::Filter()
{
    Predict();
    Correct();
    return true;
}
// earth modle and earth w will not be considered here
bool LocalizationFlow::Predict()
{
        //惯性解算
    Eigen::Vector3d pp = state_.p;
    Eigen::Vector3d vv = state_.v;
    Eigen::Quaterniond qq = state_.q;
    Eigen::Vector3d gn(0, 0, -9.809432933396721);
    Eigen::Vector3d wb;
    std::cout<<"--------------------------predict-debug-------------------------"<<std::endl;
    for (int i = 1; i < current_imu_data_.size(); ++i)
    {
        double dt = current_imu_data_[i].time - current_imu_data_[i-1].time;
        //std::cout<<"dt "<<dt<<std::endl;
        wb [0] = 0.5 * current_imu_data_[i-1].angular_velocity.x +  0.5 * current_imu_data_[i].angular_velocity.x;
        wb [1] =  0.5 * current_imu_data_[i-1].angular_velocity.y +  0.5 * current_imu_data_[i].angular_velocity.y;
        wb [2] =  0.5 * current_imu_data_[i-1].angular_velocity.z +  0.5 * current_imu_data_[i].angular_velocity.z;
        wb = wb - state_.bw;
        wb = wb * dt;
//        double angle = wb.norm();
//        Eigen::Quaterniond qb(1, 0, 0, 0);
//        if (angle != 0)
//        {
//            wb = wb / angle;
//            wb = std::sin(angle / 2) * wb;
//            qb = Eigen::Quaterniond(std::cos(angle / 2), wb[0], wb[1], wb[2]);
//        }
        double angular_delta_mag = wb.norm();
        // 方向
        Eigen::Vector3d angular_delta_dir = wb.normalized();

        // 计算四元数的增量
        double angular_delta_cos = cos(angular_delta_mag/2.0);
        double angular_delta_sin = sin(angular_delta_mag/2.0);
        Eigen::Quaterniond dq(angular_delta_cos,
                              angular_delta_sin*angular_delta_dir.x(),
                              angular_delta_sin*angular_delta_dir.y(),
                              angular_delta_sin*angular_delta_dir.z());
        Eigen::Quaterniond qq2 =(qq * dq).normalized();
        Eigen::Vector3d f1(current_imu_data_[i-1].linear_acceleration.x, current_imu_data_[i-1].linear_acceleration.y,
                           current_imu_data_[i-1].linear_acceleration.z);
        f1 = f1 -state_.ba;
        Eigen::Vector3d f2(current_imu_data_[i].linear_acceleration.x, current_imu_data_[i].linear_acceleration.y,
                           current_imu_data_[i].linear_acceleration.z);
        f2 = f2 - state_.ba;
        //std::cout<<"f2 "<<f2<<std::endl;
        Eigen::Vector3d vv2 = vv + dt * (0.5 * (qq * f1 + qq2 * f2) + gn);
        //std::cout<<"dt * (0.5 * (qq * f1 + qq2 * f2) + gn)"<<dt * (0.5 * (qq * f1 + qq2 * f2) + gn)<<std::endl;
        Eigen::Vector3d pp2 = pp + 0.5 * dt * (vv + vv2);
        //std::cout<<" 0.5 * dt * (vv + vv2)"<<0.5 * dt * (vv + vv2)<<std::endl;
        pp = pp2;
        vv = vv2;
        qq = qq2;
    }
//    std::cout<<" delta p "<<pp-state_.p<<std::endl;
//    std::cout<<" delta v "<<vv-state_.v<<std::endl;
//    std::cout<<" state_.ba "<<state_.ba<<std::endl;
    state_.p = pp;
    state_.v = vv;
    state_.q = qq;
    //////////////
    state_.p(2,0)=0;
    state_.v(2,0)=0;
    //////////////
    std::cout<<"ins calculate p"<<state_.p.transpose()<<std::endl;
    std::cout<<"ins calcalate v"<<state_.v.transpose()<<std::endl;
    std::cout<<"ins calcalate q"<<state_.q.coeffs()<<std::endl;
    Eigen::Matrix<double, 15, 15> Ft = Eigen::Matrix<double, 15, 15>::Zero();
    Ft.block<3, 3>(0, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    //Ft (3,6)
    Eigen::Matrix<double, 3, 3> temp = Eigen::Matrix<double, 3, 3>::Zero();
    Eigen::Vector3d ff(current_imu_data_.back().linear_acceleration.x, current_imu_data_.back().linear_acceleration.y,
                       current_imu_data_.back().linear_acceleration.z);
    //std::cout<<"ff"<<ff.transpose()<<std::endl;
    ff=ff-state_.ba;
    ////////////
    ff(2)=0;
    //////////////
    temp << 0,- ff[2],ff[1],ff[2],0,-ff[0],-ff[1],ff[0],0;
    temp=-qq.toRotationMatrix()*temp;
    //std::cout<<"Rt"<<qq.toRotationMatrix()<<std::endl;
    Ft.block<3, 3>(3, 6) = temp;
    //Ft(3,9)
    Ft.block<3, 3>(3, 9) = -qq.toRotationMatrix();
    //Ft(6,6)
    temp.setZero();
    wb [0] = current_imu_data_.back().angular_velocity.x;
    wb [1] =  current_imu_data_.back().angular_velocity.y;
    wb [2] =  current_imu_data_.back().angular_velocity.z;
    wb = wb - state_.bw;
    temp<<0,-wb(2),wb(1),wb(2),0,-wb(0),-wb(1),wb(0),0;
    Ft.block<3, 3>(6, 6) = -temp;
    //Ft(6,12)
    Ft.block<3, 3>(6, 12) = -Eigen::Matrix<double, 3, 3>::Identity();

    // set Bt
    Eigen::Matrix<double, 15, 12> Bt = Eigen::Matrix<double, 15, 12>::Zero();
    Bt.block<3, 3>(3, 0) = qq.toRotationMatrix();
    Bt.block<3, 3>(6, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    Bt.block<3, 3>(9, 6) = Eigen::Matrix<double, 3, 3>::Identity();
    Bt.block<3, 3>(12, 9) = Eigen::Matrix<double, 3, 3>::Identity();
    ///////////
    double T = current_imu_data_.back().time - current_imu_data_.front().time;
    //std::cout<<"Ft"<<Ft<<std::endl;
    //std::cout<<"Bt"<<Bt<<std::endl;
    Ft = Eigen::Matrix<double, 15, 15>::Identity() + Ft * T;
    Bt = Bt * T;


    Eigen::Matrix<double, 12, 1> W = Eigen::Matrix<double, 12, 1>::Zero();
    Eigen::Vector3d noise_gyro;
    noise_gyro<<gyro_noise_,gyro_noise_,gyro_noise_;
    Eigen::Vector3d noise_acc;
    noise_acc<<acc_noise_,acc_noise_,acc_noise_;
    Eigen::Vector3d noise_bias_gyro;
    noise_bias_gyro<<gyro_bias_noise_,gyro_bias_noise_,gyro_bias_noise_;
    Eigen::Vector3d noise_bias_acc;
    noise_bias_acc<<acc_bias_noise_,acc_bias_noise_,acc_bias_noise_;
    W.head(3) = noise_acc;
    W.block<3,1>(3,0)=noise_gyro;
    W.block<3,1>(6,0)=noise_bias_acc;
    W.tail(3) =noise_bias_gyro;
    /////////////////
    W(2,0)=0;
    W(5,0)=0;
    W(8,0)=0;
    W(11,0)=0;
    //////////////////
    error_state_.x = Ft * error_state_.x + Bt * W;
    ///////////
    error_state_.x(2,0)=0;
    error_state_.x(5,0)=0;
    /////////////
    std::cout<<"predict error state "<<error_state_.x.transpose()<<std::endl;
    //先验X的变化量x
    Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Identity();
    Q.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity() * acc_noise_*acc_noise_ ;
    Q.block<3, 3>(3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * gyro_noise_* gyro_noise_;
    Q.block<3, 3>(6, 6) = Eigen::Matrix<double, 3, 3>::Identity() * acc_bias_noise_ * acc_bias_noise_;
    Q.block<3, 3>(9, 9) = Eigen::Matrix<double, 3, 3>::Identity() * gyro_bias_noise_* gyro_bias_noise_;
    error_state_.p = Ft * error_state_.p * Ft.transpose() + Bt * Q * Bt.transpose();
    //先验方差p
    return true;
}
bool LocalizationFlow::Correct()
{
    bool indoor=true;
    Eigen::Matrix4d predict_pose = Eigen::Matrix4d::Identity();
    Eigen::Vector3d error_q = error_state_.x.block<3, 1>(6, 0);
    double error_q_norm = error_q.norm();
    if (error_q_norm != 0)
    {
        error_q = error_q / error_q_norm;
        error_q = error_q * std::sin(error_q_norm / 2);
    }
    Eigen::Quaterniond temp_q(std::cos(error_q_norm / 2), error_q[0], error_q[1], error_q[2]);
    predict_pose.block<3, 3>(0, 0) = ((state_.q*temp_q).normalized()).toRotationMatrix();
    predict_pose.block<3, 1>(0, 3) = state_.p+error_state_.x.block<3,1>(0,0);
    predict_pose(2,3)=0;
//    Eigen::Vector3d predict_v(0,0,0);
//    predict_v=state_.v+error_state_.x.block<3,1>(2,0);
//    predict_v(2,0)=0;
    std::cout<<"--------------------------correct-debug-------------------------"<<std::endl;

    int linked_uwb_size=current_uwb_data_.connected_nodes.size();
    std::cout<<"received all uwb stations "<<linked_uwb_size<<std::endl;

    int all_observation_size=0;
    if(0==linked_uwb_size){
        std::cout<<"no workable uwb stationes"<<std::endl;
    }
    else
    {
	

        all_observation_size=linked_uwb_size+3; // 3 for non_holonomic constrian

        Eigen::MatrixXd Y(all_observation_size,1);
        Eigen::MatrixXd Gt(all_observation_size, 15);//  Dynamic matrix
        // init Gt
        for(int m=0;m<all_observation_size;m++){
            for(int n=0;n<15;n++){
                Gt(m,n)=0;
            }
        }
        uwb_station_marker.points.clear();
        for(int j=0;j<linked_uwb_size;j++)
        {
            int current_uwb_station_id=current_uwb_data_.connected_nodes[j].id;
            Eigen::Vector3d tag_position;
            tag_position<<predict_pose(0,3),predict_pose(1,3),predict_pose(2,3);
            Eigen::Vector3d temp_uwb_dis=tag_position-init_position_class.uwb_stations_position[current_uwb_station_id];
            //std::cout<<"uwb_stations_position "<<init_position_class.uwb_stations_position[current_uwb_station_id]<<std::endl;
            //std::cout<<"tag_position "<<tag_position<<std::endl;
            //std::cout<<"temp_uwb_dis "<<temp_uwb_dis<<std::endl;
            //////////////////////
            geometry_msgs::Point p;
            p.x = init_position_class.uwb_stations_position[current_uwb_station_id](0);
            p.y = init_position_class.uwb_stations_position[current_uwb_station_id](1);
            p.z = init_position_class.uwb_stations_position[current_uwb_station_id](2);
            uwb_station_marker.points.push_back(p);
            ////////////////////////
            temp_uwb_dis(2,0)=0;////////////////////
            double prior_dis=temp_uwb_dis.norm();
            double observation_dis=current_uwb_data_.connected_nodes[j].dis;
            //std::cout<<"current_uwb_station_id "<<current_uwb_station_id<<std::endl;
            //std::cout<<"uwb_observation_dis "<<observation_dis<<std::endl;
            double delta_dis=observation_dis-prior_dis;
            Y(j, 0)=delta_dis;
            //std::cout<<"delta_dis "<<delta_dis<<std::endl;
            Eigen::Vector3d error_state_delta_pos;
            error_state_delta_pos<<error_state_.x(0,0),error_state_.x(1,0),error_state_.x(2,0);

            Eigen::Vector3d dis_vector=init_position_class.uwb_stations_position[current_uwb_station_id]
                                                -error_state_delta_pos-tag_position;
            dis_vector(2,0)=0;/////////////////////////
            Gt.block<1, 3>(j, 0)=dis_vector.transpose()*(1/dis_vector.norm());
            //std::cout<<"dis_vector.norm()"<<dis_vector.norm()<<std::endl;

        }
        uwb_station_marker_pub.publish(uwb_station_marker);

        //non holonomic constrain part
        Eigen::Vector3d R_i_z(0,0,1);
        Eigen::Vector3d R_w_z(0,0,1);
        Y.block<3, 1>(linked_uwb_size, 0)=predict_pose.block<3, 3>(0, 0)*R_i_z-R_w_z;
        Eigen::Vector3d temp_non_holo_vector= predict_pose.block<3, 3>(0, 0)*R_i_z;
        Eigen::Matrix3d temp_non_holo_matrix;
        temp_non_holo_matrix<<0,-temp_non_holo_vector(2),temp_non_holo_vector(1),temp_non_holo_vector(2),
                                0,-temp_non_holo_vector(0),-temp_non_holo_vector(1),temp_non_holo_vector(0),0;
        Gt.block<3, 3>(linked_uwb_size, 6)=temp_non_holo_matrix;

        Eigen::MatrixXd Ct(all_observation_size, all_observation_size);
        //init Ct
        for(int m=0;m<all_observation_size;m++){
            for(int n=0;n<all_observation_size;n++){
                if(m==n){
                    Ct(m,n)=1;
                }
                else{
                    Ct(m,n)=0;
                }
            }
        }

        Eigen::MatrixXd R(all_observation_size, all_observation_size);
        //init R
        for(int m=0;m<all_observation_size;m++){
            for(int n=0;n<all_observation_size;n++){
                if(m==n){
                    R(m,n)=uwb_noise_* uwb_noise_;// UWB noise
                }
                else{
                    R(m,n)=0;
                }
            }
        }
        // non_holonomic noise
        R(linked_uwb_size,linked_uwb_size)=non_hol_noise_z_*non_hol_noise_z_;
        R(linked_uwb_size+1,linked_uwb_size+1)=non_hol_noise_z_*non_hol_noise_z_;
        R(linked_uwb_size+2,linked_uwb_size+2)=non_hol_noise_z_*non_hol_noise_z_;

        Eigen::MatrixXd K(15, all_observation_size);
        std::cout<<"Y "<<Y.transpose()<<std::endl;
        //std::cout<<"Gt.transpose() "<<Gt.transpose()<<std::endl;
        //std::cout<<"Ct "<<Ct<<std::endl;
        //std::cout<<"R "<<R<<std::endl;


        K= error_state_.p * Gt.transpose()
        * (Gt * error_state_.p * Gt.transpose() + Ct * R * Ct.transpose()).inverse();
       //std::cout<<"K "<<K<<std::endl;
        error_state_.p = (Eigen::Matrix<double, 15, 15>::Identity() - K * Gt) * error_state_.p;
        error_state_.x = error_state_.x + K * (Y - Gt * error_state_.x);
        ///////////
        error_state_.x(2,0)=0;
        error_state_.x(5,0)=0;
        /////////////
        std::cout<<"corrected error state"<<error_state_.x.transpose()<<std::endl;
        state_.p = state_.p - error_state_.x.block<3, 1>(0, 0);
        //std::cout<<"corrected  state p"<<state_.p.transpose()<<std::endl;
        state_.v = state_.v - error_state_.x.block<3, 1>(3, 0);
        //std::cout<<"corrected state v"<<state_.v.transpose()<<std::endl;
        //orientation
        Eigen::Vector3d dphi_dir = error_state_.x.block<3, 1>(6, 0);
        double dphi_norm = dphi_dir.norm();
        if (dphi_norm != 0)
        {
            dphi_dir = dphi_dir / dphi_norm;
            dphi_dir = dphi_dir * std::sin(dphi_norm / 2);
        }
        Eigen::Quaterniond temp2(std::cos(dphi_norm / 2), dphi_dir[0], dphi_dir[1], dphi_dir[2]);
        state_.q = temp2 * state_.q;

        state_.ba = state_.ba - error_state_.x.block<3, 1>(9, 0);
        state_.bw = state_.bw - error_state_.x.block<3, 1>(12, 0);

        //set zero
        error_state_.x.setZero();
    }
    return true;
}
bool LocalizationFlow::PublishData() {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = state_.q.toRotationMatrix();
    pose.block<3, 1>(0, 3) = state_.p;
    std::cout<<"pose "<<pose<<std::endl;
    //tf_pub_ptr_->SendTransform(pose, current_imu_data_.time);
    odom_pub_ptr_->Publish(pose,current_uwb_data_.time);
    return true;
}
}
