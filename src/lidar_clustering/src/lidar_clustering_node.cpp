#include <lidar_clustering/lidar_clustering_node.hpp>


int main(int argc, char * argv[])
{
  const rclcpp::NodeOptions options;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Lidar_clustering>());
  rclcpp::shutdown();
  return 0;
}