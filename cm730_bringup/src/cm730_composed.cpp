#include <cm730driver/cm730driver.hpp>
#include <cm730controller/cm730controller.hpp>
#include <mx_joint_controller/mx_joint_controller.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec{};

  auto cmy730DriverNode = std::make_shared<cm730driver::Cm730Driver>();
  auto cm730ControllerNode = std::make_shared<cm730controller::Cm730Controller>();
  auto jointControllerNode = std::make_shared<mx_joint_controller::MxJointController>();

  exec.add_node(cmy730DriverNode);
  exec.add_node(cm730ControllerNode);
  exec.add_node(jointControllerNode);

  exec.spin();
  
  rclcpp::shutdown();
  cmy730DriverNode = nullptr;
  cm730ControllerNode = nullptr;
  jointControllerNode = nullptr;

  return 0;
}
