#ifndef lidar_detection__lidar_detection_NODE_HPP_
#define lidar_detection__lidar_detection_NODE_HPP_

#include "rclcpp/rclcpp.hpp"  
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "cluster_msgs/msg/cluster_array.hpp"
#include "cluster_msgs/msg/cluster.hpp"
#include "yolov8_msgs/msg/bounding_box_center_array.hpp"
#include <Eigen/Dense>
#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>
#include <cmath>
#include <vector>
#include <set>
#include <fstream>

class Lidar_detection : public rclcpp::Node
{
  public:
    Lidar_detection();

  private:
    using Policy = message_filters::sync_policies::ApproximateTime<
      yolov8_msgs::msg::BoundingBoxCenterArray, cluster_msgs::msg::ClusterArray>;

    // ROS2 subscriber and related topic name
    std::unique_ptr<message_filters::Subscriber<yolov8_msgs::msg::BoundingBoxCenterArray>>
      cam_2D_center_arr_sub_;
    std::unique_ptr<message_filters::Subscriber<cluster_msgs::msg::ClusterArray>>
      lidar_3D_center_arr_sub_;
    std::unique_ptr<message_filters::Synchronizer<Policy>> sync_;

    // ROS2 publisher and related topic name
    rclcpp::Publisher<cluster_msgs::msg::ClusterArray>::SharedPtr lidar_2D_center_arr_pub_;
    // std::string param_topic_pointcloud_out;

    void lidar_detection_cb(
      const yolov8_msgs::msg::BoundingBoxCenterArray cam_2d_center_arr_msg, 
      const cluster_msgs::msg::ClusterArray lidar_3d_center_arr_msg);

    cluster_msgs::msg::ClusterArray projection(const cluster_msgs::msg::ClusterArray lidar_3d_center_arr_msg);
    // std::vector<std::vector<double>> projection(const cluster_msgs::msg::ClusterArray lidar_3d_center_arr_msg);
    double P2 [3][4] = {
      {7.188560e+02, 0.000000e+00, 6.071928e+02, 4.538225e+01},
      {0.000000e+00, 7.188560e+02, 1.852157e+02, -1.130887e-01},
      {0.000000e+00, 0.000000e+00, 1.000000e+00, 3.779761e-03}
    };
    double R0_rect [4][4] = {
      {9.999454e-01, 7.259129e-03, -7.519551e-03, 0.000000e+00 },
      {-7.292213e-03, 9.999638e-01, -4.381729e-03, 0.000000e+00 },
      {7.487471e-03, 4.436324e-03, 9.999621e-01, 0.000000e+00 },
      {0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 }
    };
    // self.Tr_velo_to_cam = np.array([float(x) for x in calib[2].strip('\n').split(' ')[1:]]).reshape(3,4)
    // self.Tr_velo_to_cam = np.insert(self.Tr_velo_to_cam,3,values=[0,0,0,1],axis=0)
    double Tr_velo_to_cam [4][4] = {
      {7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03},
      {1.480249e-02,  7.280733e-04, -9.998902e-01, -7.631618e-02},
      {9.998621e-01,  7.523790e-03,  1.480755e-02, -2.717806e-01},
      {0.000000e+00,  0.000000e+00,  0.000000e+00,  1.000000e+00}
    };
};

#endif