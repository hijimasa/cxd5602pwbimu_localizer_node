#include "cxd5602pwbimu_localizer_node/localizer_component.hpp"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace cxd5602pwbimu_localizer_node
{
namespace
{
float hex_to_float(const std::string & hex)
{
  uint32_t raw{};
  std::stringstream ss;
  ss << std::hex << hex;
  ss >> raw;
  float val;
  std::memcpy(&val, &raw, sizeof(float));
  return val;
}
}  // namespace

LocalizerComponent::LocalizerComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("cxd5602pwbimu_localizer_component", options)
{
  serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  baud_rate_ = declare_parameter<int>("baud_rate", 115200);
  tf_parent_frame_ = declare_parameter<std::string>("tf_parent_frame", "world");
  tf_child_frame_ = declare_parameter<std::string>("tf_child_frame", "imu");
  imu_frame_ = declare_parameter<std::string>("imu_frame", "imu");

  // Allow dynamic parameter updates at runtime.
  param_cb_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult res;
      res.successful = true;
      for (const auto & p : params) {
        const auto & name = p.get_name();
        if (name == "serial_port" && p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          serial_port_ = p.as_string();
        } else if (name == "baud_rate" && p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          const int v = static_cast<int>(p.as_int());
          if (v <= 0) {
            res.successful = false;
            res.reason = "baud_rate must be positive";
            return res;
          }
          baud_rate_ = v;
        } else if (name == "tf_parent_frame" && p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          tf_parent_frame_ = p.as_string();
        } else if (name == "tf_child_frame" && p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          tf_child_frame_ = p.as_string();
        }
        else if (name == "imu_frame" && p.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
        {
          imu_frame_ = p.as_string();
        }
      }
      return res;
    });

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
  pose_pub_ = create_publisher<geometry_msgs::msg::Pose>("pose", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&LocalizerComponent::read_serial, this));

  RCLCPP_INFO(
    get_logger(), "Composable localizer ready (port=%s, baud=%d)",
    serial_port_.c_str(), baud_rate_);
}

void LocalizerComponent::publish_tf(
  const rclcpp::Time & stamp, const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = tf_parent_frame_;
  tf.child_frame_id = tf_child_frame_;
  tf.transform.translation.x = pose.position.x;
  tf.transform.translation.y = pose.position.y;
  tf.transform.translation.z = pose.position.z;
  tf.transform.rotation = pose.orientation;
  tf_broadcaster_->sendTransform(tf);
}

void LocalizerComponent::read_serial()
{
  // /dev/tty* をテキストとして読み出す
  std::ifstream stream(serial_port_);
  if (!stream.good()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Serial %s not readable", serial_port_.c_str());
    return;
  }

  std::string line;
  if (!std::getline(stream, line)) {
    return;
  }

  std::vector<std::string> parts;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) {
    if (!item.empty()) {
      parts.push_back(item);
    }
  }

  if (parts.size() != 18) {
    RCLCPP_WARN(get_logger(), "Unexpected data length: %zu", parts.size());
    return;
  }

  const auto stamp = now();

  sensor_msgs::msg::Imu imu;
  imu.header.stamp = stamp;
  imu.header.frame_id = imu_frame_;
  imu.angular_velocity.x = hex_to_float(parts[2]);
  imu.angular_velocity.y = hex_to_float(parts[3]);
  imu.angular_velocity.z = hex_to_float(parts[4]);
  imu.linear_acceleration.x = hex_to_float(parts[5]);
  imu.linear_acceleration.y = hex_to_float(parts[6]);
  imu.linear_acceleration.z = hex_to_float(parts[7]);
  imu.orientation.w = hex_to_float(parts[8]);
  imu.orientation.x = hex_to_float(parts[9]);
  imu.orientation.y = hex_to_float(parts[10]);
  imu.orientation.z = hex_to_float(parts[11]);
  imu.orientation_covariance[0] = -1.0;

  geometry_msgs::msg::Pose pose;
  pose.position.x = hex_to_float(parts[15]);
  pose.position.y = hex_to_float(parts[16]);
  pose.position.z = hex_to_float(parts[17]);
  pose.orientation = imu.orientation;

  imu_pub_->publish(imu);
  pose_pub_->publish(pose);
  publish_tf(stamp, pose);
}

}  // namespace cxd5602pwbimu_localizer_node

RCLCPP_COMPONENTS_REGISTER_NODE(cxd5602pwbimu_localizer_node::LocalizerComponent)