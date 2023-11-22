#include <distance_estimation/distance_estimation_node.hpp>


int main(int argc, char * argv[])
{
  const rclcpp::NodeOptions options;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Distance_estimation>());
  rclcpp::shutdown();
  return 0;
}