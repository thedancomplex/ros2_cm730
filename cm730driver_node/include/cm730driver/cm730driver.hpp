#ifndef CM730DRIVER__CM730DRIVER_HPP_
#define CM730DRIVER__CM730DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include "cm730driver/visibility_control.h"
#include "cm730driver_msgs/srv/ping.hpp"


namespace cm730driver
{

  class Cm730Device;
  
  class Cm730Driver : public rclcpp::Node
  {
  public:
    Cm730Driver();
    
    virtual ~Cm730Driver();

  private:
    std::shared_ptr<Cm730Device> mDevice;
    
    rclcpp::Service<cm730driver_msgs::srv::Ping>::SharedPtr mPingServer;
  };
  
}  // namespace cm730driver

#endif  // CM730DRIVER__CM730DRIVER_HPP_
