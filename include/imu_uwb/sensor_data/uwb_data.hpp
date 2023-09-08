#ifndef IMU_UWB_SENSOR_DATA_GNSS_DATA_HPP_
#define IMU_UWB_SENSOR_DATA_GNSS_DATA_HPP_

#include <mutex>
#include <thread>
#include <vector>
#include <queue>
#include <map>
#include "nlink_parser/LinktrackNode2.h"
#include "nlink_parser/LinktrackNodeframe2.h"

#include <Eigen/Dense>
#include<Eigen/Core>
namespace IMU_UWB {
class UWBData {
    public:

    void TransformCoordinate(Eigen::Matrix4f transform_matrix);

    public:
    double time=0.0;
    int id=1;
    struct ConnectedUwbNode
    {
        int role;
        int id;
        float dis;
        float fp_rssi;
        float rx_rssi;
    };
    ConnectedUwbNode connected_uwb_node;
    std::vector<ConnectedUwbNode> connected_nodes;
  
};
}
#endif