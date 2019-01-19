#include "cm730controller/cm730controller.hpp"
#include "cm730controller/cm730table.hpp"
#include "cm730controller/datautil.hpp"

using namespace std::chrono_literals;

namespace cm730controller
{

  Cm730Controller::Cm730Controller()
    : rclcpp::Node{"cm730controller"}
  {
    bulkReadClient_ = create_client<BulkRead>("bulkread");
    cm730InfoPub_ = create_publisher<CM730Info>("cm730info");
    
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
            auto cm730Result = response.get()->results[0];
            auto info = std::make_shared<CM730Info>();

            info->model_number = DataUtil::getWord(cm730Result.data, CM730Table::MODEL_NUMBER_L);
            info->version = DataUtil::getByte(cm730Result.data, CM730Table::VERSION);
            info->id = DataUtil::getByte(cm730Result.data, CM730Table::ID);
            info->baud_rate = DataUtil::getByte(cm730Result.data, CM730Table::BAUD_RATE);
            info->return_delay_time = DataUtil::getByte(cm730Result.data, CM730Table::RETURN_DELAY_TIME);
            info->return_level = DataUtil::getByte(cm730Result.data, CM730Table::RETURN_LEVEL);

            cm730InfoPub_->publish(info);
          });
      };

    loopTimer_ = create_wall_timer(8ms, loop);
  }
  
  Cm730Controller::~Cm730Controller()
  {
  }
  
}  // namespace cm730controller
