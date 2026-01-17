#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace cxd5602pwbimu_localizer_node
{

class LocalizerComponent : public rclcpp::Node
{
public:
  explicit LocalizerComponent(const rclcpp::NodeOptions & options);

private:
  void read_serial();
  void publish_tf(const rclcpp::Time & stamp, const geometry_msgs::msg::Pose & pose);

  std::string serial_port_;
  int baud_rate_{0};
  std::string tf_parent_frame_;
  std::string tf_child_frame_;
  std::string imu_frame_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace cxd5602pwbimu_localizer_node