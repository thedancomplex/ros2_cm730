#include "cm730controller/cm730controller.hpp"
#include "cm730controller/cm730table.hpp"
#include "cm730controller/datautil.hpp"

using namespace std::chrono_literals;

using namespace cm730controller_msgs::msg;

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

    // Request and wait for static info once
    bulkReadClient_->async_send_request(
      staticBulkReadRequest,
      [=](BulkReadClient::SharedFuture response) { handleStaticCm730Info(response); });
  }
  
  Cm730Controller::~Cm730Controller()
  {
  }

  void Cm730Controller::handleStaticCm730Info(BulkReadClient::SharedFuture response)
  {
    auto cm730Result = response.get()->results[0];
    staticCm730Info_ = std::make_shared<CM730EepromTable>();

    staticCm730Info_->model_number =
      DataUtil::getWord(cm730Result.data, CM730Table::MODEL_NUMBER_L);
    staticCm730Info_->version =
      DataUtil::getByte(cm730Result.data, CM730Table::VERSION);
    staticCm730Info_->id =
      DataUtil::getByte(cm730Result.data, CM730Table::ID);
    staticCm730Info_->baud_rate =
      DataUtil::getByte(cm730Result.data, CM730Table::BAUD_RATE);
    staticCm730Info_->return_delay_time =
      DataUtil::getByte(cm730Result.data, CM730Table::RETURN_DELAY_TIME);
    staticCm730Info_->return_level =
      DataUtil::getByte(cm730Result.data, CM730Table::RETURN_LEVEL);

    startLoop();
  }

  void Cm730Controller::startLoop()
  {
    auto dynamicBulkReadRequest = std::make_shared<BulkRead::Request>();
    auto cm730ReadLength = uint8_t{uint8_t(CM730Table::VOLTAGE) - uint8_t(CM730Table::DXL_POWER) + 1};
    dynamicBulkReadRequest->read_requests = {
      cm730ReadLength, 200, uint8_t(CM730Table::DXL_POWER)  // CM730 EEPROM data
    };

    auto loop =
      [=]() -> void {        
        bulkReadClient_->async_send_request(
          dynamicBulkReadRequest,
          [this](BulkReadClient::SharedFuture response) { handleDynamicCm730Info(response); });
      };

    loopTimer_ = create_wall_timer(8ms, loop);
  }

  void Cm730Controller::handleDynamicCm730Info(BulkReadClient::SharedFuture response)
  {
    if (!response.get()->success) {
      RCLCPP_ERROR(get_logger(), "Bulk read failed!");
      return;
    }
    
    auto cm730Result = response.get()->results[0];
    auto dynamicCm730Info = std::make_shared<CM730RamTable>();

    dynamicCm730Info->dynamixel_power =
      DataUtil::getByte(cm730Result.data, CM730Table::DXL_POWER);

    dynamicCm730Info->led_panel_power =
      DataUtil::getByte(cm730Result.data, CM730Table::LED_PANEL);
    dynamicCm730Info->led_5 =
      DataUtil::getWord(cm730Result.data, CM730Table::LED_5_L);
    dynamicCm730Info->led_6 =
      DataUtil::getWord(cm730Result.data, CM730Table::LED_6_L);

    dynamicCm730Info->button =
      DataUtil::getByte(cm730Result.data, CM730Table::BUTTON);

    dynamicCm730Info->gyro[0] =
      DataUtil::getWord(cm730Result.data, CM730Table::GYRO_X_L);
    dynamicCm730Info->gyro[1] =
      DataUtil::getWord(cm730Result.data, CM730Table::GYRO_Y_L);
    dynamicCm730Info->gyro[2] =
      DataUtil::getWord(cm730Result.data, CM730Table::GYRO_Z_L);

    dynamicCm730Info->accel[0] =
      DataUtil::getWord(cm730Result.data, CM730Table::ACCEL_X_L);
    dynamicCm730Info->accel[1] =
      DataUtil::getWord(cm730Result.data, CM730Table::ACCEL_Y_L);
    dynamicCm730Info->accel[2] =
      DataUtil::getWord(cm730Result.data, CM730Table::ACCEL_Z_L);

    dynamicCm730Info->voltage =
      DataUtil::getByte(cm730Result.data, CM730Table::VOLTAGE);
    
    auto cm730Info = std::make_shared<CM730Info>();
    cm730Info->stat = *staticCm730Info_;
    cm730Info->dyna = *dynamicCm730Info;    
    cm730InfoPub_->publish(cm730Info);
  }
  
}  // namespace cm730controller
