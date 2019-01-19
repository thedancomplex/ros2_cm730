#include "cm730controller/cm730controller.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<cm730controller::Cm730Controller>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  node = nullptr;

  return 0;
}
