#define BOOST_BIND_NO_PLACEHOLDERS

#include "preprocessing/preprocessing_node.hpp"

using std::placeholders::_1;
#define PI 3.14159265359
float theta_r = 180 * M_PI/ 180;

double ROI_theta(double x, double y)
{
    double r;
    double theta;

    r = sqrt((x*x)+(y*y));
    theta = acos(x/r)*180/PI;
    return theta;
}

Preprocessing::Preprocessing(const rclcpp::NodeOptions& options) : Node("preprocessing",options) 
{
  // declare_parameter<std::string>("topic_pointcloud_in","lidar_top");
  declare_parameter<std::string>("topic_pointcloud_in","kitti/velo/pointcloud");
  declare_parameter<std::string>("topic_ground_out", "preprocessing/ground");
  declare_parameter<std::string>("topic_obstacle_out", "preprocessing/obstacle");
  
  param_topic_pointcloud_in = get_parameter("topic_pointcloud_in").as_string();
  param_topic_ground_out = get_parameter("topic_ground_out").as_string();
  param_topic_obstacle_out = get_parameter("topic_obstacle_out").as_string();

  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  param_topic_pointcloud_in, 1, std::bind(&Preprocessing::preprocessing_cb, this, _1));
  pub_cloud_ground = this->create_publisher<sensor_msgs::msg::PointCloud2>(param_topic_ground_out, 1);
  pub_cloud_obstacle = this->create_publisher<sensor_msgs::msg::PointCloud2>(param_topic_obstacle_out, 1);

  RCLCPP_INFO(this->get_logger(), "\n"
  "Node:       preprocessing\n"
  "Subscribes: Pointcloud2 message: %s\n"
  "Publishes:  Pointcloud2 message: %s \n"
  "Publishes:  Pointcloud2 message: %s \n"
  "Details:    No filter applied in this example.\n"
  "Running...", param_topic_pointcloud_in.c_str(),param_topic_ground_out.c_str(),param_topic_obstacle_out.c_str());
}

pcl::PCLPointCloud2 roi(const pcl::PCLPointCloud2 cloud) 
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl1_ROI (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(cloud, *pcl1_ROI);
  pcl::PCLPointCloud2 cloud_ROI;

  for(unsigned int j=0; j<pcl1_ROI->points.size(); j++)
	{
    if(ROI_theta(pcl1_ROI->points[j].y , pcl1_ROI->points[j].x) < 45)
    {
        pcl1_ROI->points[j].x = 0;
        pcl1_ROI->points[j].y = 0;
        pcl1_ROI->points[j].z = 0;
    }
    if(ROI_theta(pcl1_ROI->points[j].y , pcl1_ROI->points[j].x) > 135)
    {
        pcl1_ROI->points[j].x = 0;
        pcl1_ROI->points[j].y = 0;
        pcl1_ROI->points[j].z = 0;
    }
    if(pcl1_ROI->points[j].x < 0)
    {
        pcl1_ROI->points[j].x = 0;
        pcl1_ROI->points[j].y = 0;
        pcl1_ROI->points[j].z = 0;
    }
  }
  pcl::toPCLPointCloud2(*pcl1_ROI, cloud_ROI);

  return cloud_ROI;
}

pcl::PCLPointCloud2 road_exstraction(const pcl::PCLPointCloud2 cloud, float road_width)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_RoadExtraction (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(cloud, *pcl_RoadExtraction);
  pcl::PCLPointCloud2 cloud_RoadExtraction;

  for(unsigned int j=0; j<pcl_RoadExtraction->points.size(); j++)
	{
    if(abs(pcl_RoadExtraction->points[j].y) > road_width/2)
    {
        pcl_RoadExtraction->points[j].x = 0;
        pcl_RoadExtraction->points[j].y = 0;
        pcl_RoadExtraction->points[j].z = 0;
    }
  }
  pcl::toPCLPointCloud2(*pcl_RoadExtraction, cloud_RoadExtraction);

  return cloud_RoadExtraction;
}

pcl::PCLPointCloud2 distance_cut(const pcl::PCLPointCloud2 cloud, float distance)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_DistanceCut (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(cloud, *pcl_DistanceCut);
  pcl::PCLPointCloud2 cloud_DistanceCut;

  for(unsigned int j=0; j<pcl_DistanceCut->points.size(); j++)
	{
    if(pcl_DistanceCut->points[j].x > distance)
    {
        pcl_DistanceCut->points[j].x = 0;
        pcl_DistanceCut->points[j].y = 0;
        pcl_DistanceCut->points[j].z = 0;
    }
  }
  pcl::toPCLPointCloud2(*pcl_DistanceCut, cloud_DistanceCut);

  return cloud_DistanceCut;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ransac(const pcl::PCLPointCloud2 cloud, const float filter_res, const Eigen::Vector4f& min_pt, const Eigen::Vector4f& max_pt)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl1_RANSAC (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(cloud, *pcl1_RANSAC);
  
  // Create the filtering object: downsample the dataset using a leaf size
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud(pcl1_RANSAC);
  vg.setLeafSize(filter_res, filter_res, filter_res);
  vg.filter(*cloud_filtered);

  // Cropping the ROI
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::CropBox<pcl::PointXYZ> region(true);
  region.setMin(min_pt);
  region.setMax(max_pt);
  region.setInputCloud(cloud_filtered);
  region.filter(*cloud_roi);

  // Removing the car roof region
  std::vector<int> indices;
  pcl::CropBox<pcl::PointXYZ> roof(true);
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
  roof.setInputCloud(cloud_roi);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  for (auto& point : indices)
    inliers->indices.push_back(point);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud_roi);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_roi);

  // pcl::PCLPointCloud2 cloud_RANSAC;
  // pcl::toPCLPointCloud2(*cloud_roi, cloud_RANSAC);

  // return cloud_RANSAC;
  return cloud_roi;
}

std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> separateClouds(const pcl::PointIndices::ConstPtr& inliers, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  // Pushback all the inliers into the ground_cloud
  for (int index : inliers->indices)
  {
    ground_cloud->points.push_back(cloud->points[index]);
  }

  // Extract the points that are not in the inliers to obstacle_cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstacle_cloud);

  return std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr>(obstacle_cloud, ground_cloud);
}

std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentPlane(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, const int max_iterations, const float distance_thresh)
{
  // Time segmentation process
  // const auto start_time = std::chrono::steady_clock::now();

  // Find inliers for the cloud.
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(max_iterations);
  seg.setDistanceThreshold(distance_thresh);

  // Segment the largest planar component from the input cloud
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.empty())
  {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  }

  return separateClouds(inliers, cloud);
}

void Preprocessing::preprocessing_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // ===========================(0) Check
  unsigned int num_points = msg->width;
  // RCLCPP_INFO(this->get_logger(), "Input Point Num: %i", num_points);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  pcl::PCLPointCloud2 cloud_transformed;
  pcl::toPCLPointCloud2(*cloud, cloud_transformed);
  // ===========================(1) ROI
  pcl::PCLPointCloud2 cloud_ROI = roi(cloud_transformed);
  // ===========================(2) Road Extraction
  pcl::PCLPointCloud2 cloud_RoadExtraction= road_exstraction(cloud_ROI, 15);
  // ===========================(3) Distance cut
  pcl::PCLPointCloud2 cloud_DistanceCut= distance_cut(cloud_RoadExtraction, 70);
  // ===========================(4) RANSAC
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_RANSAC = ransac(cloud_DistanceCut, VOXEL_GRID_SIZE, ROI_MIN_POINT, ROI_MAX_POINT);
  // ===========================(5) Separate
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_SEGMENT = segmentPlane(cloud_RANSAC, 30, GROUND_THRESH);
  // ===========================(6) Publish ground cloud and obstacle cloud
  sensor_msgs::msg::PointCloud2::Ptr ground_cloud(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*(cloud_SEGMENT.second), *ground_cloud);
  ground_cloud->header.frame_id = msg->header.frame_id;
  ground_cloud->header.stamp = msg->header.stamp;

  sensor_msgs::msg::PointCloud2::Ptr obstacle_cloud(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*(cloud_SEGMENT.first), *obstacle_cloud);
  obstacle_cloud->header.frame_id = msg->header.frame_id;
  obstacle_cloud->header.stamp = msg->header.stamp;

  pub_cloud_ground->publish(*ground_cloud);
  pub_cloud_obstacle->publish(*obstacle_cloud);
}
