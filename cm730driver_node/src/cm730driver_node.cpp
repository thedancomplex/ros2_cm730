#include "cm730driver/cm730driver.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<cm730driver::Cm730Driver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  node = nullptr;
  
  return 0;
}
