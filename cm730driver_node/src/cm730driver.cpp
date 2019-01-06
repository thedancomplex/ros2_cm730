#include "cm730driver/cm730driver.hpp"

namespace cm730driver
{

Cm730Driver::Cm730Driver()
  : rclcpp::Node{"cm730driver"}
{
  mPingServer = create_service<cm730driver_msgs::srv::Ping>(
    "ping",
    [](std::shared_ptr<rmw_request_id_t> request_header,
       std::shared_ptr<cm730driver_msgs::srv::Ping::Request> request,
       std::shared_ptr<cm730driver_msgs::srv::Ping::Response> response)
    {
      response->pong.device_id = request->ping.device_id;
    });
}

Cm730Driver::~Cm730Driver()
{
}

}  // namespace cm730driver
