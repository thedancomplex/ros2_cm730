#ifndef CM730CONTROLLER__CM730CONTROLLER_HPP_
#define CM730CONTROLLER__CM730CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <cm730driver_msgs/srv/write.hpp>
#include <cm730driver_msgs/srv/bulk_read.hpp>
#include <cm730driver_msgs/srv/sync_write.hpp>
#include <cm730controller_msgs/msg/cm730_info.hpp>
#include <cm730controller_msgs/msg/mx28_info_array.hpp>
#include <cm730controller_msgs/msg/mx28_command_array.hpp>

#include "cm730controller/visibility_control.h"

namespace cm730controller
{

  class Cm730Controller : public rclcpp::Node
  {
  public:
    Cm730Controller();
    
    virtual ~Cm730Controller();

  private:
    // Convenience types
    using Write = cm730driver_msgs::srv::Write;
    using BulkRead = cm730driver_msgs::srv::BulkRead;
    using SyncWrite = cm730driver_msgs::srv::SyncWrite;
    
    using CM730Info = cm730controller_msgs::msg::CM730Info;
    using MX28InfoArray = cm730controller_msgs::msg::MX28InfoArray;
    using MX28CommandArray = cm730controller_msgs::msg::MX28CommandArray;
    
    using CM730EepromTable = cm730controller_msgs::msg::CM730EepromTable;
    using MX28EepromTable = cm730controller_msgs::msg::MX28EepromTable;

    using WriteClient = rclcpp::Client<Write>;
    using BulkReadClient = rclcpp::Client<BulkRead>;
    using SyncWriteClient = rclcpp::Client<SyncWrite>;

    // Clients for CM730 driver services
    WriteClient::SharedPtr writeClient_;
    BulkReadClient::SharedPtr bulkReadClient_;
    SyncWriteClient::SharedPtr syncWriteClient_;
    
    // Subscribers
    rclcpp::Subscription<MX28CommandArray>::SharedPtr mx28CommandSub_;

    std::mutex mx28CommandMutex_;
    MX28CommandArray::SharedPtr mx28Command_;
    
    // Publishers
    rclcpp::Publisher<CM730Info>::SharedPtr cm730InfoPub_;
    rclcpp::Publisher<MX28InfoArray>::SharedPtr mx28InfoPub_;

    // Static info
    CM730EepromTable::SharedPtr staticCm730Info_;
    std::map<uint8_t, MX28EepromTable::SharedPtr> staticMx28Info_;

    // Timer for main loop
    rclcpp::TimerBase::SharedPtr loopTimer_;
    
    void powerOn();
    void readStaticInfo();
    void handleStaticInfo(BulkReadClient::SharedFuture response);
    void startLoop();
    void handleDynamicInfo(BulkReadClient::SharedFuture response);
    void writeCommands();

    template<typename TCommand>
    typename TCommand::SharedPtr grabCommand(typename TCommand::SharedPtr& cmd, std::mutex& mutex) {
      std::lock_guard<std::mutex> lock{mutex};
      if (cmd == nullptr)
        return nullptr;
      auto grab = cmd;
      cmd.reset();
      return grab;
    }
  };
  
}  // namespace cm730controller

#endif  // CM730CONTROLLER__CM730CONTROLLER_HPP_
