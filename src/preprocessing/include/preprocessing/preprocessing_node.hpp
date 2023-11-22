// Copyright (c) 2022 Jonas Mahler

// This file is part of Preprocessing.

// Preprocessing is free software: you can redistribute it and/or modify it under the terms 
// of the GNU General Public License as published by the Free Software Foundation, 
// either version 3 of the License, or (at your option) any later version.

// Preprocessing is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
// See the GNU General Public License for more details.

// You should have received a copy of the GNU General Public License along 
// with Preprocessing. If not, see <https://www.gnu.org/licenses/>. 

#ifndef preprocessing__preprocessing_NODE_HPP_
#define preprocessing__preprocessing_NODE_HPP_

#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>
#include <cmath>
#include <vector>
#include <set>
#include <utility>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <boost/format.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>

#include "rclcpp/rclcpp.hpp"  
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "cluster_msgs/msg/cluster_array.hpp"


class Preprocessing : public rclcpp::Node
{
  public:

    explicit Preprocessing(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~Preprocessing() {};

  protected:
    // Pointcloud Filtering Parameters
    float VOXEL_GRID_SIZE = 0.2;
    Eigen::Vector4f ROI_MAX_POINT = Eigen::Vector4f(70, 30, 1, 1);
    Eigen::Vector4f ROI_MIN_POINT = Eigen::Vector4f(-30, -30, -2.5, 1);
    float GROUND_THRESH = 0.3;

    void preprocessing_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    // void publishClouds(const std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr>&& segmented_clouds, const std_msgs::msg::Header& header);
    // ROS2 subscriber and related topic name
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::string param_topic_pointcloud_in;
    
    
    // ROS2 publisher and related topic name   
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    // std::string param_topic_pointcloud_out;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_ground;
    std::string param_topic_ground_out;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_obstacle;
    std::string param_topic_obstacle_out;
};

#endif