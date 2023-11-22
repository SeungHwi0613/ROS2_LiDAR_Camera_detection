#include <lidar_detection/lidar_detection_node.hpp>


int main(int argc, char * argv[])
{
  const rclcpp::NodeOptions options;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Lidar_detection>());
  rclcpp::shutdown();
  return 0;
}