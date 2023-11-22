#include <comparison/comparison_node.hpp>


int main(int argc, char * argv[])
{
  const rclcpp::NodeOptions options;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Comparison>());
  rclcpp::shutdown();
  return 0;
}