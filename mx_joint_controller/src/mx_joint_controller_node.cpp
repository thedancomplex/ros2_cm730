#include "mx_joint_controller/mx_joint_controller.hpp"
#include <cstdio>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<mx_joint_controller::MxJointController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  node = nullptr;

  return 0;
}
