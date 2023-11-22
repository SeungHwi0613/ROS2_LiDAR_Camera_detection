#ifndef comparison__comparison_NODE_HPP_
#define comparison__comparison_NODE_HPP_

#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>
#include <cmath>
#include <vector>
#include <set>
#include "rclcpp/rclcpp.hpp"  
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "yolov8_msgs/msg/bounding_box_center_array.hpp"
#include "cluster_msgs/msg/cluster.hpp"
#include "cluster_msgs/msg/cluster_array.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>


class Comparison : public rclcpp::Node
{
  public:
    
    explicit Comparison(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~Comparison() {};

  private:
    using Policy = message_filters::sync_policies::ApproximateTime<
      yolov8_msgs::msg::BoundingBoxCenterArray, cluster_msgs::msg::ClusterArray, cluster_msgs::msg::ClusterArray>;
    
    double slope = -0.117; //-11/94
    double yIntercept = 44.0;

    // ROS2 subscriber and related topic name
    std::unique_ptr<message_filters::Subscriber<yolov8_msgs::msg::BoundingBoxCenterArray>>
      cam_sub_;
    std::unique_ptr<message_filters::Subscriber<cluster_msgs::msg::ClusterArray>>
      lidar2D_sub_;
    std::unique_ptr<message_filters::Subscriber<cluster_msgs::msg::ClusterArray>>
      lidar3D_sub_;
    std::unique_ptr<message_filters::Synchronizer<Policy>> sync_;

    // ROS2 publisher and related topic name
    rclcpp::Publisher<cluster_msgs::msg::ClusterArray>::SharedPtr pub_;
    // std::string param_topic_pointcloud_out;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    void comparison_cb(
      const yolov8_msgs::msg::BoundingBoxCenterArray cam_msg, 
      const cluster_msgs::msg::ClusterArray lidar_msg,
      const cluster_msgs::msg::ClusterArray lidar3D_msg);

    std::pair<std::vector<std::pair<double, double>>,std::vector<std::pair<double, double>>> sort_vector(
      const yolov8_msgs::msg::BoundingBoxCenterArray cam_msg, 
      const cluster_msgs::msg::ClusterArray lidar_msg);

    std::pair<std::pair<double, double>, std::pair<double, double>> distance_cal(
      std::pair<double, double>cam_center, 
      std::vector<std::pair<double, double>>lidar_sorted_centers);

    double extract_distance(
      std::pair<double, double> lidar_matched_centers, 
      cluster_msgs::msg::ClusterArray lidar_msg,
      cluster_msgs::msg::ClusterArray lidar3D_msg);

    std::vector<std::pair<double, double>> single_camera_geometry(
      std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> first_second_closest,
      cluster_msgs::msg::ClusterArray lidar_msg,
      cluster_msgs::msg::ClusterArray lidar3D_msg);

    std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> closest_matching(
      std::vector<std::pair<double, double>>cam_sorted_centers, 
      std::vector<std::pair<double, double>>lidar_sorted_centers);

    void lidar_centers_publish(
      const cluster_msgs::msg::ClusterArray lidar_msg, 
      const cluster_msgs::msg::ClusterArray lidar3D_msg, 
      std::vector<std::pair<double, double>> lidar_matched_centers);

    void visualization(cluster_msgs::msg::ClusterArray lidar_centers);

};

#endif