#define BOOST_BIND_NO_PLACEHOLDERS

#include "comparison/comparison_node.hpp"

std::pair<std::vector<std::pair<double, double>>, std::vector<std::pair<double, double>>> Comparison::sort_vector(
  const yolov8_msgs::msg::BoundingBoxCenterArray cam_msg, 
  const cluster_msgs::msg::ClusterArray lidar_msg) 
{
  std::pair<std::vector<std::pair<double, double>>, std::vector<std::pair<double, double>>> cam_lidar_sorted_centers;
  std::vector<std::pair<double, double>> cam_unsorted_centers;
  std::vector<std::pair<double, double>> lidar_unsorted_centers;
  std::vector<std::pair<double, double>> cam_sorted_centers;
  std::vector<std::pair<double, double>> lidar_sorted_centers;

  // <-----------------------cam sort----------------------->
  int cam_len = cam_msg.centers.size();
  for(int cam=0; cam<cam_len; cam++)
  {
    std::pair<double, double> cam_center = std::make_pair(cam_msg.centers[cam].x_center, cam_msg.centers[cam].y_center);
    cam_unsorted_centers.push_back(cam_center);
  }
  std::sort(cam_unsorted_centers.begin(), cam_unsorted_centers.end());
  cam_sorted_centers=cam_unsorted_centers;
 
  // <-----------------------lidar sort----------------------->
  int lidar_len = lidar_msg.centers.size();
  for(int lidar=0; lidar<lidar_len; lidar++)
  {
    std::pair<double, double> lidar_center = std::make_pair(lidar_msg.centers[lidar].x_center, lidar_msg.centers[lidar].y_center);
    lidar_unsorted_centers.push_back(lidar_center);
  }
  std::sort(lidar_unsorted_centers.begin(), lidar_unsorted_centers.end());
  lidar_sorted_centers=lidar_unsorted_centers;

  // <---------------------cam_lidar pair--------------------->
  cam_lidar_sorted_centers=std::make_pair(cam_sorted_centers, lidar_sorted_centers);

  return cam_lidar_sorted_centers;
}

std::pair<std::pair<double, double>, std::pair<double, double>> Comparison:: distance_cal(
  std::pair<double, double>cam_center, 
  std::vector<std::pair<double, double>>lidar_sorted_centers)
{
  // distance between [target camera center] and [whole lidar clustering centers] 
  int lidar_len=lidar_sorted_centers.size();
  
  std::vector<double> distance_vec;
  for(int lidar=0; lidar<lidar_len; lidar++)
  {
    double cam_x=cam_center.first;
    double cam_y=cam_center.second;
    double lidar_x=lidar_sorted_centers[lidar].first;
    double lidar_y=lidar_sorted_centers[lidar].second;
    
    double distance=sqrt(pow(cam_x-lidar_x,2)+pow(cam_y-lidar_y,2));

    distance_vec.push_back(distance);

  }

  std::vector<double> original_distance_vec=distance_vec;

  // first minimum
  auto first_min = std::min_element(distance_vec.begin(), distance_vec.end());
  int first_min_id = std::distance(distance_vec.begin(), first_min);

  // second minimum
  std::vector<double>::iterator it = distance_vec.begin() + 1;
  std::nth_element(distance_vec.begin(), it, distance_vec.end());
  double second_min_id_ordered = std::distance(distance_vec.begin(), it);
  
  double target_value = distance_vec[second_min_id_ordered];
  auto it_ = std::find(original_distance_vec.begin(), original_distance_vec.end(), target_value);
  int second_min_id;
  if (it_ != original_distance_vec.end()) {
      second_min_id = std::distance(original_distance_vec.begin(), it_);
  } else {
      std::cout << "Cannot find " << target_value << " from original distance vector" << std::endl;
  }

  // ((first_min_x, first_min_y), (second_min_x, second_min_y))
  std::pair<std::pair<double, double>, std::pair<double, double>> first_second_closest;
  if(original_distance_vec[first_min_id]<50 && original_distance_vec[second_min_id]<50){
    first_second_closest=std::make_pair(lidar_sorted_centers[first_min_id], lidar_sorted_centers[second_min_id]);
  }
  else{
    std::cout<<"first error: "<<original_distance_vec[first_min_id]<<std::endl;
    std::cout<<"second error: "<<original_distance_vec[second_min_id]<<std::endl;
  }
  
  return first_second_closest;
}

double Comparison::extract_distance(
  std::pair<double, double> lidar_matched_centers, 
  cluster_msgs::msg::ClusterArray lidar_msg, 
  cluster_msgs::msg::ClusterArray lidar3D_msg)
{
  // based on 2D x, y coordinate, searching cluster id of lidar_msg
  int lidar_msg_len = lidar_msg.centers.size();
  for(int lidar_msg_id=0; lidar_msg_id<lidar_msg_len; lidar_msg_id++)
  {


    bool x_match;
    bool y_match;
    // 2D_x matching
    if(lidar_msg.centers[lidar_msg_id].x_center==lidar_matched_centers.first)
    {
      x_match=true;
      // 2D_y matching
      if(lidar_msg.centers[lidar_msg_id].y_center==lidar_matched_centers.second)
      {
        y_match=true;
      } else{  y_match=false;  }
    } else{  x_match=false;  }

    // exracting cluster id
    if(x_match && y_match){
      int matched_id=lidar_msg.centers[lidar_msg_id].id;

      // exracting 3D coordinate
      for(int i=0; i<lidar3D_msg.centers.size(); i++)
      {
        if(lidar3D_msg.centers[i].id==matched_id){
          double target_distance=lidar3D_msg.centers[i].x_center;
          return target_distance;
        }
      }
    }


  }
}

std::vector<std::pair<double, double>> Comparison::single_camera_geometry(
  std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> first_second_centers, 
  cluster_msgs::msg::ClusterArray lidar_msg, 
  cluster_msgs::msg::ClusterArray lidar3D_msg)
{
  std::vector<std::pair<double, double>> final_closest;
  for(int i=0; i<first_second_centers.size(); i++)
  {
    // calculate distance based on LU equation using single camera geometry [criteria]
    double first_y=first_second_centers[i].first.second;
    double first_dist=slope*first_y+yIntercept;

    double second_y=first_second_centers[i].second.second;
    double second_dist=slope*second_y+yIntercept;

    // extract distance based on LiDAR clustering [target]
    double first_cluster_dist=extract_distance(first_second_centers[i].first, lidar_msg, lidar3D_msg);
    double second_cluster_dist=extract_distance(first_second_centers[i].second, lidar_msg, lidar3D_msg);

    double first_dist_error=fabs(first_cluster_dist-first_dist);
    double second_dist_error=fabs(second_cluster_dist-second_dist);
    // std::cout<<"first_dist: "<<first_dist<<"  ||  "<<"second_dist: "<<second_dist<<std::endl;
    // std::cout<<"first_cluster_dist: "<<first_cluster_dist<<"  ||  "<<"second_cluster_dist: "<<second_cluster_dist<<std::endl;
    // std::cout<<"first_dist_error: "<<first_dist_error<<"  ||  "<<"second_dist_error: "<<second_dist_error<<std::endl;

    if(first_dist_error<second_dist_error){
      final_closest.push_back(first_second_centers[i].first);
    }
    else{
      final_closest.push_back(first_second_centers[i].second);
    }
  }
  return final_closest;
}

std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> Comparison::closest_matching(
  std::vector<std::pair<double, double>>cam_sorted_centers, 
  std::vector<std::pair<double, double>>lidar_sorted_centers)
{
  int cam_len=cam_sorted_centers.size();
  std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> first_second_centers;

  for(int cam=0; cam<cam_len; cam++)
  {
    std::pair<std::pair<double, double>, std::pair<double, double>> first_second_closest=distance_cal(cam_sorted_centers[cam], lidar_sorted_centers);
    first_second_centers.push_back(first_second_closest);
  }
  return first_second_centers;
}


Comparison::Comparison(const rclcpp::NodeOptions& options) 
: Node("comparison",options)
{  
 /*related topics */
  cam_sub_ =
    std::make_unique<message_filters::Subscriber<yolov8_msgs::msg::BoundingBoxCenterArray>>(
      this, "/yolo/bbox_center2d_arr");
  lidar2D_sub_ =
    std::make_unique<message_filters::Subscriber<cluster_msgs::msg::ClusterArray>>(
      this, "/lidar_detection/center2d_arr");
  lidar3D_sub_ =
    std::make_unique<message_filters::Subscriber<cluster_msgs::msg::ClusterArray>>(
      this, "/lidar_clustering/center3d_arr");

  sync_ =
    std::make_unique<message_filters::Synchronizer<Policy>>(
      Policy(10), *cam_sub_, *lidar2D_sub_, *lidar3D_sub_);

  sync_->registerCallback(&Comparison::comparison_cb, this);

  pub_ = this->create_publisher<cluster_msgs::msg::ClusterArray>("/comparison/final", 1);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization/marker", 1);
 
  RCLCPP_INFO(this->get_logger(), "\n"
  "Node:       Comparison\n");
}

void Comparison::comparison_cb(
  const yolov8_msgs::msg::BoundingBoxCenterArray cam_msg, 
  const cluster_msgs::msg::ClusterArray lidar_msg,
  const cluster_msgs::msg::ClusterArray lidar3D_msg) 
{
  std::cout<<"YOLO_len: "<<cam_msg.centers.size()<<std::endl;
  // std::cout<<"||   callBack   ||"<<std::endl;
  // sorted vector for algorithm
  std::pair<std::vector<std::pair<double, double>>, std::vector<std::pair<double, double>>> cam_lidar_sorted_centers;
  cam_lidar_sorted_centers=sort_vector(cam_msg, lidar_msg);
  std::vector<std::pair<double, double>> cam_sorted_centers=cam_lidar_sorted_centers.first;
  std::vector<std::pair<double, double>> lidar_sorted_centers=cam_lidar_sorted_centers.second;

  // closest point searching (first, second)
  std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> first_second_centers;
  first_second_centers=closest_matching(cam_sorted_centers, lidar_sorted_centers);

  // distance correct matching (final)
  std::vector<std::pair<double, double>> final_closest;
  final_closest=single_camera_geometry(first_second_centers, lidar_msg, lidar3D_msg);

  // publish as /comparison/final && visualize_msg/MarkArray
  lidar_centers_publish(lidar_msg, lidar3D_msg, final_closest);
}

void Comparison::lidar_centers_publish(
  const cluster_msgs::msg::ClusterArray lidar_msg, 
  const cluster_msgs::msg::ClusterArray lidar3D_msg, 
  std::vector<std::pair<double, double>> lidar_matched_centers)
{
  cluster_msgs::msg::ClusterArray lidar_centers;

  int lidar_msg_len = lidar_msg.centers.size();

  int lidar_vec_len = lidar_matched_centers.size();
  
  for(int lidar_vec_id=0; lidar_vec_id<lidar_vec_len; lidar_vec_id++)
  {
    cluster_msgs::msg::Cluster lidar_center;
    for(int lidar_msg_id=0; lidar_msg_id<lidar_msg_len; lidar_msg_id++)
    {
      bool x_match;
      bool y_match;

      if(lidar_msg.centers[lidar_msg_id].x_center==lidar_matched_centers[lidar_vec_id].first)
      {
        x_match=true;
        if(lidar_msg.centers[lidar_msg_id].y_center==lidar_matched_centers[lidar_vec_id].second)
        {
          y_match=true;
        } else{  y_match=false;  }
      } else{  x_match=false;  }

      if(x_match && y_match){
        lidar_center.header=lidar_msg.header;
        int matched_id=lidar_msg.centers[lidar_msg_id].id;

        for(int i=0; i<lidar3D_msg.centers.size(); i++)
        {
          if(lidar3D_msg.centers[i].id==matched_id){
            lidar_center.id=matched_id;
            lidar_center.x_center=lidar3D_msg.centers[i].x_center;
            lidar_center.y_center=lidar3D_msg.centers[i].y_center;
            lidar_center.z_center=lidar3D_msg.centers[i].z_center;
          }
        }
        lidar_centers.centers.push_back(lidar_center);
      }
    }
    lidar_centers.header=lidar_msg.header;
  }
  visualization(lidar_centers);
  pub_->publish(lidar_centers);
}

void Comparison::visualization(cluster_msgs::msg::ClusterArray lidar_centers)
{
  auto marker_array_msg = std::make_shared<visualization_msgs::msg::MarkerArray>();

  int lidar_centers_len=lidar_centers.centers.size();
  std::vector<double> rand_color1={1.0, 0.0, 0.0};
  std::vector<double> rand_color2={0.0, 1.0, 0.0};
  std::vector<double> rand_color3={0.0, 0.0, 1.0};
  std::srand(static_cast<unsigned int>(std::time(nullptr)));
  int random_index = std::rand() % rand_color1.size();



  // 예시로 두 개의 마커 생성
  // for (int i = 0; i < 2; ++i) {
  // std::cout<<"Marker Length: "<<lidar_centers_len<<std::endl;
  if(lidar_centers_len>0){
    for (int i = 0; i < lidar_centers_len; ++i) {  
      if(!std::isnan(lidar_centers.centers[i].x_center) && !std::isnan(lidar_centers.centers[i].y_center) && !std::isnan(lidar_centers.centers[i].z_center)
      && fabs(lidar_centers.centers[i].x_center)<900 && fabs(lidar_centers.centers[i].y_center)<900 && fabs(lidar_centers.centers[i].z_center)<900
      && lidar_centers.centers[i].z_center<0){
        // std::cout<<"Marker value check_x: "<<lidar_centers.centers[i].x_center<<std::endl;
        // std::cout<<"Marker value check_y: "<<lidar_centers.centers[i].y_center<<std::endl;
        // std::cout<<"Marker value check_z: "<<lidar_centers.centers[i].z_center<<std::endl;
        auto marker_msg = std::make_shared<visualization_msgs::msg::Marker>();
        marker_msg->header=lidar_centers.header;
        marker_msg->id = i;
        marker_msg->type = visualization_msgs::msg::Marker::SPHERE;  // 마커 타입 설정
        marker_msg->action = visualization_msgs::msg::Marker::ADD;

        marker_msg->pose.position.x = lidar_centers.centers[i].x_center;  // 마커 위치 설정
        marker_msg->pose.position.y = lidar_centers.centers[i].y_center;
        marker_msg->pose.position.z = lidar_centers.centers[i].z_center;

        marker_msg->pose.orientation.x = 0.0;
        marker_msg->pose.orientation.y = 0.0;
        marker_msg->pose.orientation.z = 0.0;
        marker_msg->pose.orientation.w = 1.0;
        
        marker_msg->scale.x = 1;  // 마커 크기 설정 (0.3 best)
        marker_msg->scale.y = 1;
        marker_msg->scale.z = 1;
        marker_msg->color.r = rand_color1[random_index];  // 마커 색상 설정
        marker_msg->color.g = rand_color2[random_index];
        marker_msg->color.b = rand_color3[random_index];
        marker_msg->color.a = 1.0;
        marker_msg->lifetime = rclcpp::Duration(0.5, 0);  // 마커 유지 시간 설정

        marker_array_msg->markers.push_back(*marker_msg);
        marker_msg->action = visualization_msgs::msg::Marker::DELETEALL;
      }
      else{
        std::cout<<"Nan value"<<std::endl;
      }
    }

    // 마커 배열 메시지 발행
    std::cout<<"Marker_len: "<<marker_array_msg->markers.size()<<std::endl;
    marker_pub_->publish(*marker_array_msg);
  }
  else{
    std::cout<<"No markers"<<std::endl;
  }

}
