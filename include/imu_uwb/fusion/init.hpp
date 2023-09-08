#ifndef IMU_UWB_INIT_HPP_
#define IMU_UWB_INIT_HPP_

#include <string>
#include <cmath>
#include <Eigen/Dense>
#include<Eigen/Core>

namespace IMU_UWB
{
    class InitPosition{
        public:
            struct point_t
        {
            double x, y;
        };

        struct circle_t
        {
            struct point_t center;
            double r;
        };
            struct point
        {
            double x, y;
        };
        Eigen::Vector3d init_position;
        std::vector<Eigen::Vector3d> uwb_stations_position;
        std::string data_dir_path_="/home/xc/2021-0916/result_data";
        void ReadUwbPosion();
        Eigen::Vector3d InitUwbTagPosition(double dis_0, double dis_1, double dis_2,int uwb_id_0, int uwb_id_1, int uwb_id_2);
        // catculate x,y,z 
        int double_equals(double a, double b);
        double distance_sqr(struct point_t* a, struct point_t* b);
        double distance(struct point_t* a, struct point_t* b);
        int insect(struct circle_t circles[], struct point_t points[]);
        void Cross_Point(struct circle_t circles[], struct point_t Location[]);
        double norm(struct point p);
        Eigen::Vector3d trilateration(struct point point1, struct point point2, struct point point3, double r1, double r2, double r3);

    };
};
#endif
