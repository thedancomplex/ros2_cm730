#include "cm730controller/cm730controller.hpp"
#include "cm730controller/cm730table.hpp"
#include "cm730controller/mx28table.hpp"
#include "cm730controller/datautil.hpp"

using namespace std::chrono_literals;
using namespace cm730controller_msgs::msg;

namespace cm730controller
{

  Cm730Controller::Cm730Controller()
    : rclcpp::Node{"cm730controller"}
  {
    writeClient_ = create_client<Write>("/cm730/write");
    bulkReadClient_ = create_client<BulkRead>("/cm730/bulkread");
    
    cm730InfoPub_ = create_publisher<CM730Info>("/cm730/cm730info");
    mx28InfoPub_ = create_publisher<MX28InfoArray>("/cm730/mx28info");
    
    while (!writeClient_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Bulk read client interrupted while waiting for service to appear.");
        return;
      }
      RCLCPP_INFO(get_logger(), "Waiting for CM730 driver to appear...");
    }
    
    powerOn();
  }
  
  Cm730Controller::~Cm730Controller()
  {
  }

  void Cm730Controller::powerOn() {
    auto powerOnRequest = std::make_shared<Write::Request>();
    powerOnRequest->device_id = 200;
    powerOnRequest->address = uint8_t(CM730Table::DXL_POWER);
    powerOnRequest->data = {1};

    RCLCPP_INFO(get_logger(), "Powering on DXL bus...");
    writeClient_->async_send_request(
      powerOnRequest,
      [=](WriteClient::SharedFuture response) {
        if (!response.get()->success) {
          RCLCPP_ERROR(get_logger(), "Failed powering on DXL bus");
        } else {
          readStaticInfo();
        }
      });
  }

  void Cm730Controller::readStaticInfo()
  {
    // Prepare bulk read request messages for reading static information,
    auto staticBulkReadRequest = std::make_shared<BulkRead::Request>();
    staticBulkReadRequest->read_requests = {
      uint8_t(CM730Table::EEPROM_LENGTH), 200, 0  // CM730 EEPROM data
    };

    for (auto i = 1; i <= 20; ++i) {
      staticBulkReadRequest->read_requests.push_back(uint8_t(MX28Table::EEPROM_LENGTH));
      staticBulkReadRequest->read_requests.push_back(i);
      staticBulkReadRequest->read_requests.push_back(0);
    }
    
    // Request and wait for static info once
    RCLCPP_INFO(get_logger(), "Reading static info...");
    bulkReadClient_->async_send_request(
      staticBulkReadRequest,
      [=](BulkReadClient::SharedFuture response) { handleStaticInfo(response); });
  }
  
  void Cm730Controller::handleStaticInfo(BulkReadClient::SharedFuture response)
  {
    auto const& results = response.get()->results;
    RCLCPP_INFO(get_logger(), "Received static CM730 info; # of results: " + std::to_string(results.size()));

    // CM730
    auto cm730Result = results[0];
    staticCm730Info_ = std::make_shared<CM730EepromTable>();

    staticCm730Info_->model_number =
      DataUtil::getWord(cm730Result.data, CM730Table::MODEL_NUMBER_L, CM730Table::MODEL_NUMBER_L);
    staticCm730Info_->version =
      DataUtil::getByte(cm730Result.data, CM730Table::VERSION, CM730Table::MODEL_NUMBER_L);
    staticCm730Info_->id =
      DataUtil::getByte(cm730Result.data, CM730Table::ID, CM730Table::MODEL_NUMBER_L);
    staticCm730Info_->baud_rate =
      DataUtil::getByte(cm730Result.data, CM730Table::BAUD_RATE, CM730Table::MODEL_NUMBER_L);
    staticCm730Info_->return_delay_time =
      DataUtil::getByte(cm730Result.data, CM730Table::RETURN_DELAY_TIME, CM730Table::MODEL_NUMBER_L);
    staticCm730Info_->return_level =
      DataUtil::getByte(cm730Result.data, CM730Table::RETURN_LEVEL, CM730Table::MODEL_NUMBER_L);

    staticMx28Info_.clear();
    for (auto i = 1; i <= 20; ++i) {
      auto mx28Result = results[i];
      assert(mx28Result.device_id == i);
      auto staticMx28Info = std::make_shared<MX28EepromTable>();

      staticMx28Info->model_number =
        DataUtil::getWord(mx28Result.data, MX28Table::MODEL_NUMBER_L, MX28Table::MODEL_NUMBER_L);
      staticMx28Info->version =
        DataUtil::getByte(mx28Result.data, MX28Table::VERSION, MX28Table::MODEL_NUMBER_L);
      staticMx28Info->id =
        DataUtil::getByte(mx28Result.data, MX28Table::ID, MX28Table::MODEL_NUMBER_L);
      staticMx28Info->baud_rate =
        DataUtil::getByte(mx28Result.data, MX28Table::BAUD_RATE, MX28Table::MODEL_NUMBER_L);
      staticMx28Info->return_delay_time =
        DataUtil::getByte(mx28Result.data, MX28Table::RETURN_DELAY_TIME, MX28Table::MODEL_NUMBER_L);
      staticMx28Info->cw_angle_limit =
        DataUtil::getWord(mx28Result.data, MX28Table::CW_ANGLE_LIMIT_L, MX28Table::MODEL_NUMBER_L);
      staticMx28Info->ccw_angle_limit =
        DataUtil::getWord(mx28Result.data, MX28Table::CCW_ANGLE_LIMIT_L, MX28Table::MODEL_NUMBER_L);
      staticMx28Info->high_limit_temperature =
        DataUtil::getByte(mx28Result.data, MX28Table::HIGH_LIMIT_TEMPERATURE, MX28Table::MODEL_NUMBER_L);
      staticMx28Info->low_limit_voltage =
        DataUtil::getByte(mx28Result.data, MX28Table::LOW_LIMIT_VOLTAGE, MX28Table::MODEL_NUMBER_L);
      staticMx28Info->high_limit_voltage =
        DataUtil::getByte(mx28Result.data, MX28Table::HIGH_LIMIT_VOLTAGE, MX28Table::MODEL_NUMBER_L);
      staticMx28Info->max_torque =
        DataUtil::getWord(mx28Result.data, MX28Table::MAX_TORQUE_L, MX28Table::MODEL_NUMBER_L);
      staticMx28Info->return_level =
        DataUtil::getByte(mx28Result.data, MX28Table::RETURN_LEVEL, MX28Table::MODEL_NUMBER_L);
      staticMx28Info->alarm_led =
        DataUtil::getByte(mx28Result.data, MX28Table::ALARM_LED, MX28Table::MODEL_NUMBER_L);
      staticMx28Info->alarm_shutdown =
        DataUtil::getByte(mx28Result.data, MX28Table::ALARM_SHUTDOWN, MX28Table::MODEL_NUMBER_L);
      staticMx28Info->operating_mode =
        DataUtil::getByte(mx28Result.data, MX28Table::OPERATING_MODE, MX28Table::MODEL_NUMBER_L);
      staticMx28Info->low_calibration =
        DataUtil::getWord(mx28Result.data, MX28Table::LOW_CALIBRATION_L, MX28Table::MODEL_NUMBER_L);
      staticMx28Info->high_calibration =
        DataUtil::getWord(mx28Result.data, MX28Table::HIGH_CALIBRATION_L, MX28Table::MODEL_NUMBER_L);
        
      staticMx28Info_[i] = staticMx28Info;
    }
    startLoop();
  }

  void Cm730Controller::startLoop()
  {
    auto dynamicBulkReadRequest = std::make_shared<BulkRead::Request>();
    auto cm730ReadLength = uint8_t{uint8_t(CM730Table::VOLTAGE) - uint8_t(CM730Table::DXL_POWER) + 1};
    dynamicBulkReadRequest->read_requests = {
      cm730ReadLength, 200, uint8_t(CM730Table::DXL_POWER)  // CM730 EEPROM data
    };

    auto mx28ReadLength = uint8_t{uint8_t(MX28Table::PRESENT_TEMPERATURE) - uint8_t(MX28Table::PRESENT_POSITION_L) + 1};
    for (auto i = uint8_t{1}; i <= 20; ++i) {
      dynamicBulkReadRequest->read_requests.insert(
        dynamicBulkReadRequest->read_requests.end(), {
          mx28ReadLength, i, uint8_t(MX28Table::PRESENT_POSITION_L)
        });
    }

    auto loop =
      [=]() -> void {        
        bulkReadClient_->async_send_request(
          dynamicBulkReadRequest,
          [this](BulkReadClient::SharedFuture response) { handleDynamicInfo(response); });
      };

    loopTimer_ = create_wall_timer(8ms, loop);
  }

  void Cm730Controller::handleDynamicInfo(BulkReadClient::SharedFuture response)
  {
    if (!response.get()->success) {
      RCLCPP_ERROR(get_logger(), "Bulk read failed!");
      return;
    }
    
    auto cm730Result = response.get()->results[0];
    auto dynamicCm730Info = std::make_shared<CM730RamTable>();

    dynamicCm730Info->dynamixel_power =
      DataUtil::getByte(cm730Result.data, CM730Table::DXL_POWER, CM730Table::DXL_POWER);

    dynamicCm730Info->led_panel_power =
      DataUtil::getByte(cm730Result.data, CM730Table::LED_PANEL, CM730Table::DXL_POWER);
    dynamicCm730Info->led_5 =
      DataUtil::getWord(cm730Result.data, CM730Table::LED_5_L, CM730Table::DXL_POWER);
    dynamicCm730Info->led_6 =
      DataUtil::getWord(cm730Result.data, CM730Table::LED_6_L, CM730Table::DXL_POWER);

    dynamicCm730Info->button =
      DataUtil::getByte(cm730Result.data, CM730Table::BUTTON, CM730Table::DXL_POWER);

    dynamicCm730Info->gyro[0] =
      DataUtil::getWord(cm730Result.data, CM730Table::GYRO_X_L, CM730Table::DXL_POWER);
    dynamicCm730Info->gyro[1] =
      DataUtil::getWord(cm730Result.data, CM730Table::GYRO_Y_L, CM730Table::DXL_POWER);
    dynamicCm730Info->gyro[2] =
      DataUtil::getWord(cm730Result.data, CM730Table::GYRO_Z_L, CM730Table::DXL_POWER);

    dynamicCm730Info->accel[0] =
      DataUtil::getWord(cm730Result.data, CM730Table::ACCEL_X_L, CM730Table::DXL_POWER);
    dynamicCm730Info->accel[1] =
      DataUtil::getWord(cm730Result.data, CM730Table::ACCEL_Y_L, CM730Table::DXL_POWER);
    dynamicCm730Info->accel[2] =
      DataUtil::getWord(cm730Result.data, CM730Table::ACCEL_Z_L, CM730Table::DXL_POWER);

    dynamicCm730Info->voltage =
      DataUtil::getByte(cm730Result.data, CM730Table::VOLTAGE, CM730Table::DXL_POWER);
    
    auto cm730Info = std::make_shared<CM730Info>();
    cm730Info->stat = *staticCm730Info_;
    cm730Info->dyna = *dynamicCm730Info;    
    cm730InfoPub_->publish(cm730Info);

    auto mx28Infos = std::make_shared<MX28InfoArray>();
    for (auto i = 1; i <= 20; ++i) {
      auto mx28Result = response.get()->results[i];
      auto dynamicMx28Info = std::make_shared<MX28RamTable>();

      dynamicMx28Info->present_position =
        DataUtil::getWord(mx28Result.data, MX28Table::PRESENT_POSITION_L, MX28Table::PRESENT_POSITION_L);
      dynamicMx28Info->present_speed =
        DataUtil::getWord(mx28Result.data, MX28Table::PRESENT_SPEED_L, MX28Table::PRESENT_POSITION_L);
      dynamicMx28Info->present_voltage =
        DataUtil::getByte(mx28Result.data, MX28Table::PRESENT_VOLTAGE, MX28Table::PRESENT_POSITION_L);
      dynamicMx28Info->present_temperature =
        DataUtil::getByte(mx28Result.data, MX28Table::PRESENT_TEMPERATURE, MX28Table::PRESENT_POSITION_L);
      
      auto mx28Info = std::make_shared<MX28Info>();
      mx28Info->stat = *staticMx28Info_[mx28Result.device_id];
      mx28Info->dyna = *dynamicMx28Info;
      mx28Infos->mx28s.push_back(*mx28Info);
    }
    mx28InfoPub_->publish(mx28Infos);
  }
  
}  // namespace cm730controller
