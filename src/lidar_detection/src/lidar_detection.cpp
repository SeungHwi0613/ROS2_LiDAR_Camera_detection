#define BOOST_BIND_NO_PLACEHOLDERS

#include "lidar_detection/lidar_detection_node.hpp"

Lidar_detection::Lidar_detection() 
: Node("lidar_detection")
{  
  /*related topics */
  cam_2D_center_arr_sub_ =
    std::make_unique<message_filters::Subscriber<yolov8_msgs::msg::BoundingBoxCenterArray>>(
      this, "/yolo/bbox_center2d_arr");
  lidar_3D_center_arr_sub_ =
    std::make_unique<message_filters::Subscriber<cluster_msgs::msg::ClusterArray>>(
      this, "/lidar_clustering/center3d_arr");

  sync_ =
    std::make_unique<message_filters::Synchronizer<Policy>>(
      Policy(10), *cam_2D_center_arr_sub_, *lidar_3D_center_arr_sub_);

  sync_->registerCallback(&Lidar_detection::lidar_detection_cb, this);

  lidar_2D_center_arr_pub_ = this->create_publisher<cluster_msgs::msg::ClusterArray>("/lidar_detection/center2d_arr", 1);
 
  RCLCPP_INFO(this->get_logger(), "\n"
  "Node:       Lidar_detection\n");
}

cluster_msgs::msg::ClusterArray Lidar_detection::projection(const cluster_msgs::msg::ClusterArray lidar_3d_center_arr_msg)
// std::vector<std::vector<double>> Lidar_detection::projection(const cluster_msgs::msg::ClusterArray lidar_3d_center_arr_msg)
{
  // #<------------------Calibration file load------------------>
  // return; 
  cluster_msgs::msg::ClusterArray lidar_2d_center_arr;
  // std::vector<std::vector<double>> img;
  lidar_2d_center_arr.header=lidar_3d_center_arr_msg.header;

  int cluster_arr_len = lidar_3d_center_arr_msg.centers.size();
  // int each_cluster=0;
  if(cluster_arr_len >0)
  {
    for(int each_cluster=0; each_cluster<=cluster_arr_len; each_cluster++)
    {
      double x=lidar_3d_center_arr_msg.centers[each_cluster].x_center;
      double y=lidar_3d_center_arr_msg.centers[each_cluster].y_center;
      double z=lidar_3d_center_arr_msg.centers[each_cluster].z_center;
      // [[x]
      //  [y]
      //  [z]] 
      Eigen::MatrixXd points;
      points = Eigen::MatrixXd(3, 1);
      points(0, 0) = x;
      points(1, 0) = y;
      points(2, 0) = z;

      // [[x]
      //  [y]
      //  [z]
      //  [1]] 
      Eigen::MatrixXd velo(points.rows() + 1, points.cols());
      velo.topRows(points.rows()) = points;
      velo.row(points.rows()) << 1;

      // velo = np.delete(velo,np.where(velo[0,:]<0),axis=1)
      for(int i = velo.cols() - 1; i >= 0; --i) {
          if(velo(0, i) < 0) {
              velo.block(0, i, velo.rows(), velo.cols() - i - 1) = velo.block(0, i + 1, velo.rows(), velo.cols() - i - 1);
              velo.conservativeResize(velo.rows(), velo.cols() - 1);
          }
      }

      // Convert array of parameter to matrix
      Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> P2_mat(P2[0]);
      Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> R0_rect_mat(R0_rect[0]);
      Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> Tr_velo_to_cam_mat(Tr_velo_to_cam[0]);
      
      // Calculation
      Eigen::MatrixXd cam = P2_mat * (R0_rect_mat * (Tr_velo_to_cam_mat * velo));
      // <----------------------Compared with original python code result---------------------->and exactly same[O]
      // std::cout << "P2_mat:" << P2_mat << std::endl;
      // std::cout << "R0_rect_mat:" << R0_rect_mat << std::endl;
      // std::cout << "Tr_velo_to_cam_mat:"<< Tr_velo_to_cam_mat << std::endl;

      // cam = np.delete(cam,np.where(cam[2,:]<0),axis=1)
      for(int i = cam.cols() - 1; i >= 0; --i) {
          if(cam(2, i) < 0) {
              cam.block(0, i, cam.rows(), cam.cols() - i - 1) = cam.block(0, i + 1, cam.rows(), cam.cols() - i - 1);
              cam.conservativeResize(cam.rows(), cam.cols() - 1);
          }
      }

      // cam[:2] /= cam[2,:]
      for (int i=0; i<cam.cols(); ++i){
        cam.col(i).segment(0, 2) /= cam(2,i);
      }

      if (cam.size()>0)
      {
        cluster_msgs::msg::Cluster lidar_2d_center;
        lidar_2d_center.header=lidar_3d_center_arr_msg.header;
        lidar_2d_center.id=lidar_3d_center_arr_msg.centers[each_cluster].id;
        lidar_2d_center.x_center=cam.row(0)(0);
        lidar_2d_center.y_center=cam.row(1)(0);
        lidar_2d_center.z_center=lidar_3d_center_arr_msg.centers[each_cluster].z_center;
        lidar_2d_center_arr.centers.push_back(lidar_2d_center);
      }
      // each_cluster++;
    } 
  }
  return lidar_2d_center_arr;
}

void Lidar_detection::lidar_detection_cb(
  const yolov8_msgs::msg::BoundingBoxCenterArray cam_2d_center_arr_msg, 
  const cluster_msgs::msg::ClusterArray lidar_3d_center_arr_msg) 
{
  // std::cout<<"CallBack"<<std::endl;
  cluster_msgs::msg::ClusterArray lidar_2d_center_arr_msg = projection(lidar_3d_center_arr_msg);
  lidar_2D_center_arr_pub_->publish(lidar_2d_center_arr_msg);
  // std::cout<<"check_after:"<<lidar_3d_center_arr_msg.centers.size()<<"=="<<lidar_2d_center_arr_msg.centers.size()<<std::endl; //different[v]

}