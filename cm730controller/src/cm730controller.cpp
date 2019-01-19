#include "cm730controller/cm730controller.hpp"
#include "cm730controller/cm730table.hpp"

using namespace std::chrono_literals;

namespace cm730controller
{

  Cm730Controller::Cm730Controller()
    : rclcpp::Node{"cm730controller"}
  {
    bulkReadClient_ = create_client<cm730driver_msgs::srv::BulkRead>("bulkread");

    // Prepare bulk read request messages for reading static information,
    // and for reading dynamic information
    auto staticBulkReadRequest = std::make_shared<BulkRead::Request>();
    staticBulkReadRequest->read_requests = {
      uint8_t(CM730Table::EEPROM_LENGTH), 200, 0  // CM730 EEPROM data
    };

    auto loop =
      [=]() -> void {        
        bulkReadClient_->async_send_request(
          staticBulkReadRequest,
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
