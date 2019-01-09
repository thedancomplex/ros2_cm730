#ifndef CM730DRIVER__CM730DRIVER_HPP_
#define CM730DRIVER__CM730DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include "cm730driver/visibility_control.h"
#include "cm730driver_msgs/srv/ping.hpp"
#include "cm730driver_msgs/srv/read.hpp"


namespace cm730driver
{

  class Cm730Device;
  class PingService;
  class ReadService;
  
  class Cm730Driver : public rclcpp::Node
  {
  public:
    Cm730Driver();
    
    virtual ~Cm730Driver();

  private:
    std::shared_ptr<Cm730Device> mDevice;

    std::tuple<std::shared_ptr<PingService>,
               rclcpp::Service<cm730driver_msgs::srv::Ping>::SharedPtr> mPingServer;
    std::tuple<std::shared_ptr<ReadService>,
               rclcpp::Service<cm730driver_msgs::srv::Read>::SharedPtr> mReadServer;
  };
  
}  // namespace cm730driver

#endif  // CM730DRIVER__CM730DRIVER_HPP_
