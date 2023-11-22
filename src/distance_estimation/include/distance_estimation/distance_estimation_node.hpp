#ifndef distance_estimation__distance_estimation_NODE_HPP_
#define distance_estimation__distance_estimation_NODE_HPP_

#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>
#include <cmath>
#include <vector>
#include <set>
#include <list>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"  
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"
#include "cluster_msgs/msg/cluster.hpp"
#include "cluster_msgs/msg/cluster_array.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>


class Distance_estimation : public rclcpp::Node
{
  public:
    
    explicit Distance_estimation(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~Distance_estimation() {};

  private:
    using Policy = message_filters::sync_policies::ApproximateTime<
      yolov8_msgs::msg::DetectionArray, cluster_msgs::msg::ClusterArray>;

    // ROS2 subscriber and related topic name
    std::unique_ptr<message_filters::Subscriber<yolov8_msgs::msg::DetectionArray>>
      detection_sub_;
    std::unique_ptr<message_filters::Subscriber<cluster_msgs::msg::ClusterArray>>
      center_sub_;
    std::unique_ptr<message_filters::Synchronizer<Policy>> sync_;

    std::vector<float> prev_frame_track_id_array;
    std::vector<std::vector<float>> prev_frame_total_array;

    int frame_i=0;

    float most_common_search(std::vector<float> prev_frame_track_id_array);
    float distance_estimation(float most_common_id, std::vector<std::vector<float>> prev_frame_total_array);

    void distance_estimation_cb(
      const yolov8_msgs::msg::DetectionArray detection_msg, 
      const cluster_msgs::msg::ClusterArray center_msg);

};

#endif