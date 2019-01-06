#ifndef CM730DRIVER__CM730DRIVER_HPP_
#define CM730DRIVER__CM730DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include "cm730driver/visibility_control.h"

namespace cm730driver
{

  class Cm730Driver : public rclcpp::Node
  {
  public:
    Cm730Driver();
    
    virtual ~Cm730Driver();
  };
  
}  // namespace cm730driver

#endif  // CM730DRIVER__CM730DRIVER_HPP_
