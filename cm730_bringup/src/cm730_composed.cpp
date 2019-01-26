#include <cm730driver/cm730driver.hpp>
#include <cm730controller/cm730controller.hpp>
#include <mx_joint_state_publisher/mx_joint_state_publisher.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec{};

  auto driverNode = std::make_shared<cm730driver::Cm730Driver>();
  auto controllerNode = std::make_shared<cm730controller::Cm730Controller>();
  auto jointStatePublisherNode = std::make_shared<mx_joint_state_publisher::MxJointStatePublisher>();

  exec.add_node(driverNode);
  exec.add_node(controllerNode);
  exec.add_node(jointStatePublisherNode);

  exec.spin();
  
  rclcpp::shutdown();
  driverNode = nullptr;
  controllerNode = nullptr;
  jointStatePublisherNode = nullptr;

  return 0;
}
