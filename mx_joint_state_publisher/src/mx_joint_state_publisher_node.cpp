#include "mx_joint_state_publisher/mx_joint_state_publisher.hpp"
#include <cstdio>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<mx_joint_state_publisher::MxJointStatePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  node=nullptr;

  return 0;
}
