#ifndef CM730CONTROLLER__CM730CONTROLLER_HPP_
#define CM730CONTROLLER__CM730CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <cm730driver_msgs/srv/bulk_read.hpp>

#include "cm730controller/visibility_control.h"

namespace cm730controller
{

  class Cm730Controller : public rclcpp::Node
  {
  public:
    using BulkRead = cm730driver_msgs::srv::BulkRead;
    using BulkReadClient = rclcpp::Client<BulkRead>;
    
    Cm730Controller();
    
    virtual ~Cm730Controller();

  private:
    rclcpp::Client<BulkRead>::SharedPtr bulkReadClient_;
    rclcpp::TimerBase::SharedPtr loopTimer_;
  };
  
}  // namespace cm730controller

#endif  // CM730CONTROLLER__CM730CONTROLLER_HPP_
