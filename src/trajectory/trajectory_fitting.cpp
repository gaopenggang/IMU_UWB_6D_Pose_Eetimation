#include "imu_uwb/trajectory/trajectory_fitting.hpp"
#include<string>
namespace IMU_UWB {
    void Trajectory::ReadTrajectory() {
        std::string pg_path = data_dir_path_ + "/pose_graph/pose_graph.txt";
        std::FILE *pg_ifile = std::fopen(pg_path.c_str(), "r");
        if (pg_ifile == NULL) {
            std::cout << "Error ! Open [ " << pg_path << " ] failed ..." << std::endl;
            return;
        }
        int i;
        double t,px, py, pz, qx, qy, qz, qw;
        //std::string t;
        while (std::fscanf(pg_ifile, "%d %lf %lf %lf %lf %lf %lf %lf %lf", &i, &t, &px, &py, &pz, &qx, &qy, &qz, &qw) !=
               EOF) {
            //std::cout << "i " << i << std::endl;
            trajectory_pose.t(0) = px;
            trajectory_pose.t(1) = py;
            trajectory_pose.t(2) = 0;
            //std::cout<<"ref traj id "<<i<<std::endl;
            //std::cout<<"ref traj "<<trajectory_pose.t.transpose()<<std::endl;
            Eigen::Quaterniond temp_q(qw, qx, qy, qz);
            trajectory_pose.q = temp_q;
            ros::Time ros_time((double) t);
            trajectory_pose.uwb_time = ros_time.toSec();
            //std::cout.precision(64);
            //std::cout<<"traj time"<<trajectory_pose.uwb_time<<std::endl;
            trajectory_poses.push_back(trajectory_pose);
        }
        std::fclose(pg_ifile);
        std::cout <<"successful read trajectory"<<std::endl;
    }

// catculate the pose and velocity of the nearest tarjectory segmentation
    Eigen::Matrix<double, 1, 10> Trajectory::CalculateObservation(int id) {
        //std::cout<<"trajectory_poses[id].t"<<trajectory_poses[id].t<<std::endl;
        observation.block<1, 3>(0, 0) = trajectory_poses[id].t;

        Eigen::Vector3d v;
        if(0==id)
        {
            v<<0,0,0;
        }
        else
        {
            //std::cout.precision(64);
            //std::cout<<"time"<<(trajectory_poses[id].uwb_time - trajectory_poses[id-1].uwb_time)<<std::endl;
            //std::cout<<"trajectory_poses[id].uwb_time"<<trajectory_poses[id].uwb_time<<std::endl;
            //std::cout<<"trajectory_poses[id-1].uwb_time"<<trajectory_poses[id-1].uwb_time<<std::endl;
            v = (trajectory_poses[id].t - trajectory_poses[id-1].t) /
                (trajectory_poses[id].uwb_time - trajectory_poses[id-1].uwb_time);
            std::cout<<"v= "<<v<<std::endl;
        }
        observation.block<1, 3>(0, 3) = v;
        observation(0,5)=0;
        //observation.block<1, 3>(0, 6) = trajectory_poses[id].q;

        return observation;

    }

    int Trajectory::CalculateNearestPoseID(Eigen::Vector3d position) {
        int pose_size = trajectory_poses.size();
        int nearest_id = 0;
        double nearest_dis = 1000000000;

        for (int i = 0; i < pose_size; i++) {
            double dx = position(0) - trajectory_poses[i].t(0);
            double dy = position(1) - trajectory_poses[i].t(1);
            //double dz = position(2) - trajectory_poses[i].t(2);
            double dis = sqrt(dx * dx + dy * dy );
            if (dis < nearest_dis) {
                nearest_dis=dis;
                nearest_id = i;
            }
        }
        return nearest_id;
    }

}