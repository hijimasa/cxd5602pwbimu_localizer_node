#include <rclcpp/rclcpp.hpp>

#include "cxd5602pwbimu_localizer_node/localizer_component.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cxd5602pwbimu_localizer_node::LocalizerComponent>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}