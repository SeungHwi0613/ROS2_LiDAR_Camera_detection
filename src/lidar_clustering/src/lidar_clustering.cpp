#define BOOST_BIND_NO_PLACEHOLDERS

#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>
#include <cmath>
#include <vector>
#include <set>

#include "lidar_clustering/lidar_clustering_node.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
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


using std::placeholders::_1;

Lidar_clustering::Lidar_clustering(const rclcpp::NodeOptions& options) 
: Node("lidar_clustering",options),
  tf2_buffer(this->get_clock()),
  tf2_listener(tf2_buffer)
{  
  declare_parameter<std::string>("topic_pointcloud_in","preprocessing/obstacle");
  declare_parameter<std::string>("topic_pointcloud_out", "lidar_clustering/clustering");
  declare_parameter<std::string>("topic_centroid_out", "lidar_clustering/center3d_arr");

  param_topic_pointcloud_in = get_parameter("topic_pointcloud_in").as_string();
  param_topic_pointcloud_out = get_parameter("topic_pointcloud_out").as_string();
  param_topic_centroid_out = get_parameter("topic_centroid_out").as_string();

  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_out, 1);
  centroid_pub_ = this->create_publisher<cluster_msgs::msg::ClusterArray>(param_topic_centroid_out, 1);
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  param_topic_pointcloud_in, 1, std::bind(&Lidar_clustering::lidar_clustering_cb, this, _1));

  RCLCPP_INFO(this->get_logger(), "\n"
  "Node:       lidar_clustering\n"
  "Subscribes: Pointcloud2 message: %s\n"
  "Publishes:  Pointcloud2 message: %s \n"
  "Details:    No filter applied in this example.\n"
  "Running...", param_topic_pointcloud_in.c_str(),param_topic_pointcloud_out.c_str());
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Lidar_clustering::clustering(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, const float cluster_tolerance, const int min_size, const int max_size, std_msgs::msg::Header header)
{
  // Perform euclidean clustering to group detected obstacles
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_size);
  ec.setMaxClusterSize(max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  // Color visualization
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_clustered(new pcl::PointCloud<pcl::PointXYZI>);

  int j = 0;
  bool frame_end;
  cluster_msgs::msg::ClusterArray cluster_arr_msg;
  int id=0;
  // Checking the each cluster!!!!
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++, j++) 
  {   
    // Checking the each point!!!! in a same cluster. And maknig it having own color for visualization 
    pcl::PointCloud<pcl::PointXYZI>::Ptr unit_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    frame_end=false;
    // if(it->indices.back()<100) //if more than 100, it's not a vehicle

    for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) 
    {
      if(it->indices.size()<POINTS_NUM_LIMIT){
        // std::cout<<"cluster_size: "<<it->indices.size()<<std::endl;
        pcl::PointXYZI tmp;
        tmp.x = cloud->points[*pit].x;
        tmp.y = cloud->points[*pit].y;
        tmp.z = cloud->points[*pit].z;
        tmp.intensity = j%8;
        unit_cluster->points.push_back(tmp);
        cloud_clustered->points.push_back(tmp);
      }
      else{
        // std::cout<<"Maybe wall, tree or something not important"<<std::endl;
        break;
      }
    }
    
    // cluster_msgs::msg::Cluster cluster;
    if(it==cluster_indices.end()-1){
      frame_end=true;
    }
    
    // std::cout<<"cluster_points_size: "<<unit_cluster->points.size()<<std::endl; //if more than 100, it's not a vehicle
    cluster_arr_msg = calculate_centroid(id, cluster_arr_msg,unit_cluster,frame_end,header); 
    id++;
  }
  cloud_clustered->width = cloud_clustered->points.size();
  cloud_clustered->height = 1;
  cloud_clustered->is_dense = true;

  if(frame_end==true){
    cluster_arr_msg.header=header;
    // cluster_arr_msg.frame_end=frame_end;
    centroid_pub_->publish(cluster_arr_msg);
  }
  return cloud_clustered;
}

cluster_msgs::msg::ClusterArray Lidar_clustering::calculate_centroid(int id, cluster_msgs::msg::ClusterArray cluster_arr_msg, pcl::PointCloud<pcl::PointXYZI>::Ptr unit_cluster, bool frame_end, std_msgs::msg::Header header)
{
  cluster_msgs::msg::Cluster cluster;

  Eigen::Matrix<float, 4, 1> centroid;
  pcl::compute3DCentroid(*unit_cluster, centroid);
  cluster.header=header;
  cluster.id=id;
  cluster.x_center = centroid[0];
  cluster.y_center = centroid[1];
  cluster.z_center = centroid[2]; 
  

  cluster_arr_msg.centers.push_back(cluster);

  return cluster_arr_msg;
}

void Lidar_clustering::lidar_clustering_cb(const sensor_msgs::msg::PointCloud2::SharedPtr ros_pc2_in) 
{

  // std::cout<<"||||   Callback START   ||||"<<std::endl;
  unsigned int num_points = ros_pc2_in->width;
  // RCLCPP_INFO(this->get_logger(), "Input Point Num: %i", num_points);

  /*** Convert ROS message to PCL ***/
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*ros_pc2_in, *pcl_pc_in);
  
  // Cluster objects
  auto cloud_clusters = clustering(pcl_pc_in, CLUSTER_THRESH, CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE, ros_pc2_in->header);
  // Calculate centroid
  // auto cluster_centroids = calculate_centroid(cloud_clusters);

  // ===========================(6) Publish ground cloud and obstacle cloud
  sensor_msgs::msg::PointCloud2::Ptr cluster_cloud(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*cloud_clusters, *cluster_cloud);
  cluster_cloud->header.frame_id = ros_pc2_in->header.frame_id;
  cluster_cloud->header.stamp = ros_pc2_in->header.stamp;

  publisher_->publish(*cluster_cloud);
}