#ifndef CM730CONTROLLER__CM730CONTROLLER_HPP_
#define CM730CONTROLLER__CM730CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <cm730driver_msgs/srv/write.hpp>
#include <cm730driver_msgs/srv/bulk_read.hpp>
#include <cm730controller_msgs/msg/cm730_info.hpp>

#include "cm730controller/visibility_control.h"

namespace cm730controller
{

  class Cm730Controller : public rclcpp::Node
  {
  public:
    Cm730Controller();
    
    virtual ~Cm730Controller();

  private:
    using Write = cm730driver_msgs::srv::Write;
    using BulkRead = cm730driver_msgs::srv::BulkRead;
    using CM730Info = cm730controller_msgs::msg::CM730Info;
    using CM730EepromTable = cm730controller_msgs::msg::CM730EepromTable;
    
    using WriteClient = rclcpp::Client<Write>;
    using BulkReadClient = rclcpp::Client<BulkRead>;

    WriteClient::SharedPtr writeClient_;
    BulkReadClient::SharedPtr bulkReadClient_;
    
    rclcpp::TimerBase::SharedPtr loopTimer_;
    
    std::shared_ptr<rclcpp::Publisher<CM730Info>> cm730InfoPub_;

    CM730EepromTable::SharedPtr staticCm730Info_;

    void powerOn();
    void readStaticInfo();
    void handleStaticInfo(BulkReadClient::SharedFuture response);
    void handleDynamicCm730Info(BulkReadClient::SharedFuture response);
    void startLoop();
  };
  
}  // namespace cm730controller

#endif  // CM730CONTROLLER__CM730CONTROLLER_HPP_
