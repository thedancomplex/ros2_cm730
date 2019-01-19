#include "cm730controller/cm730controller.hpp"

using namespace std::chrono_literals;

namespace cm730controller
{

  Cm730Controller::Cm730Controller()
    : rclcpp::Node{"cm730controller"}
  {
    bulkReadClient_ = create_client<cm730driver_msgs::srv::BulkRead>("bulkread");
    auto loop =
      [this]() -> void {
        auto bulkReadRequest = std::make_shared<BulkRead::Request>();
        bulkReadRequest->read_requests = {
          17, 200, 0  // First 17 bytes of CM730 control register
        };
        
        bulkReadClient_->async_send_request(
          bulkReadRequest,
          [this](BulkReadClient::SharedFuture response) {
            RCLCPP_INFO(get_logger(), "Number of results in response: " + std::to_string(response.get()->results.size()));
          });
      };

    loopTimer_ = create_wall_timer(8ms, loop);
  }
  
  Cm730Controller::~Cm730Controller()
  {
  }
  
}  // namespace cm730controller
