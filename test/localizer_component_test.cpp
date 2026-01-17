#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "cxd5602pwbimu_localizer_node/localizer_component.hpp"

class LocalizerComponentTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(LocalizerComponentTest, DefaultParameters)
{
  auto node = std::make_shared<cxd5602pwbimu_localizer_node::LocalizerComponent>(
    rclcpp::NodeOptions());

  std::string serial_port;
  int64_t baud_rate{};
  std::string tf_parent;
  std::string tf_child;
  std::string imu_frame;

  ASSERT_TRUE(node->get_parameter("serial_port", serial_port));
  ASSERT_TRUE(node->get_parameter("baud_rate", baud_rate));
  ASSERT_TRUE(node->get_parameter("tf_parent_frame", tf_parent));
  ASSERT_TRUE(node->get_parameter("tf_child_frame", tf_child));
  ASSERT_TRUE(node->get_parameter("imu_frame", imu_frame));

  EXPECT_EQ("/dev/ttyUSB0", serial_port);
  EXPECT_EQ(115200, baud_rate);
  EXPECT_EQ("world", tf_parent);
  EXPECT_EQ("imu", tf_child);
  EXPECT_EQ("imu", imu_frame);
}

TEST_F(LocalizerComponentTest, RejectNegativeBaudRate)
{
  auto node = std::make_shared<cxd5602pwbimu_localizer_node::LocalizerComponent>(
    rclcpp::NodeOptions());

  auto result = node->set_parameter(rclcpp::Parameter("baud_rate", -9600));
  EXPECT_FALSE(result.successful);

  int64_t baud_rate{};
  ASSERT_TRUE(node->get_parameter("baud_rate", baud_rate));
  EXPECT_EQ(115200, baud_rate);
}

TEST_F(LocalizerComponentTest, UpdateFrameParameters)
{
  auto node = std::make_shared<cxd5602pwbimu_localizer_node::LocalizerComponent>(
    rclcpp::NodeOptions());

  std::vector<rclcpp::Parameter> params{
    rclcpp::Parameter("tf_parent_frame", "map"),
    rclcpp::Parameter("tf_child_frame", "imu_link"),
    rclcpp::Parameter("imu_frame", "imu_sensor")};

  const auto results = node->set_parameters(params);
  for (const auto & res : results) {
    EXPECT_TRUE(res.successful);
  }

  std::string tf_parent;
  std::string tf_child;
  std::string imu_frame;

  ASSERT_TRUE(node->get_parameter("tf_parent_frame", tf_parent));
  ASSERT_TRUE(node->get_parameter("tf_child_frame", tf_child));
  ASSERT_TRUE(node->get_parameter("imu_frame", imu_frame));

  EXPECT_EQ("map", tf_parent);
  EXPECT_EQ("imu_link", tf_child);
  EXPECT_EQ("imu_sensor", imu_frame);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
