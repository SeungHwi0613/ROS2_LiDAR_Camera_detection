#ifndef lidar_clustering__lidar_clustering_NODE_HPP_
#define lidar_clustering__lidar_clustering_NODE_HPP_

#include "rclcpp/rclcpp.hpp"  
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "cluster_msgs/msg/cluster.hpp"
#include "cluster_msgs/msg/cluster_array.hpp"
// #include "cluster_msgs/msg/detected_object.hpp"
// #include "cluster_msgs/msg/detected_object_array.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

class Lidar_clustering : public rclcpp::Node
{
  public:
    
    explicit Lidar_clustering(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~Lidar_clustering() {};

  protected:
    size_t obstacle_id_=0;
    // std::vector<Box> prev_boxes_, curr_boxes_;
    tf2_ros::Buffer tf2_buffer;
    tf2_ros::TransformListener tf2_listener;
    std::string bbox_target_frame_, bbox_source_frame_;
    bool USE_PCA_BOX=false;
    bool USE_TRACKING=true;
    float CLUSTER_THRESH=0.6;
    int CLUSTER_MAX_SIZE=5000;
    int CLUSTER_MIN_SIZE=10;
    float DISPLACEMENT_THRESH=1.0;
    float IOU_THRESH=1.0;
    int POINTS_NUM_LIMIT=200;

    void lidar_clustering_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustering(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, const float cluster_tolerance, const int min_size, const int max_size, std_msgs::msg::Header header);
    cluster_msgs::msg::ClusterArray calculate_centroid(int id, cluster_msgs::msg::ClusterArray cluster_arr_msg, pcl::PointCloud<pcl::PointXYZI>::Ptr unit_cluster, bool frame_end, std_msgs::msg::Header header);

    // ROS2 subscriber and related topic name
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::string param_topic_pointcloud_in;
    
    // ROS2 publisher and related topic name   
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::string param_topic_pointcloud_out;

    // ROS2 publisher and related topic name   
    rclcpp::Publisher<cluster_msgs::msg::ClusterArray>::SharedPtr centroid_pub_;
    std::string param_topic_centroid_out;
};

#endif