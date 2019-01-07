#include "cm730driver/cm730driver.hpp"

#include "cm730driver/cm730device.hpp"
#include "cm730driver/pingservice.hpp"

#include <numeric>

using namespace std::literals::chrono_literals;

namespace cm730driver
{
  
  Cm730Driver::Cm730Driver()
    : rclcpp::Node{"cm730driver"}
  {
    mDevice = std::make_shared<Cm730Device>("/dev/ttyUSB0");
    mDevice->open();

    mPingServer = PingService::create(*this, "ping", mDevice, get_clock());

    /*
    mPingServer = create_service<cm730driver_msgs::srv::Ping>(
      "ping",
      [this](std::shared_ptr<cm730driver_msgs::srv::Ping::Request> request,
         std::shared_ptr<cm730driver_msgs::srv::Ping::Response> response)
      {
        mDevice->clear();
        
        auto data = std::array<uint8_t, 6>{
          0xFF, 0xFF,  // prefix
          request->ping.device_id,  // device ID
          2,  // length
          1,  // instruction
          0,  // checksum
        };

        auto sum = std::accumulate(std::next(data.begin(), 2), std::prev(data.end(), 1), 0u);
        data[5] = ~sum;
        mDevice->write(data.data(), data.size());
        RCLCPP_INFO(get_logger(), "Wrote ping");

        auto readStartTime = now();
        auto nRead = 0;
        while (nRead < data.size())
        {
          auto readingTime = now() - readStartTime;
          if (readingTime.nanoseconds() / 1e6 > 12)
          {
            RCLCPP_INFO(get_logger(), "Timed out");
            response->pong.success = false;
            return;
          }
          
          auto n = mDevice->read(data.data() + nRead, data.size() - nRead);
          if (n > 0)
          {
            nRead += n;
            auto str = std::ostringstream{};
            str << "Total read: " << nRead << " - ";
            for (auto i = 0; i < nRead; ++i)
              str << int{data[i]} << " ";
            RCLCPP_INFO(get_logger(), str.str());
          }
          rclcpp::sleep_for(100us);
        }
        
        response->pong.success = true;
      });
    */
  }

  Cm730Driver::~Cm730Driver()
  {
    mDevice->close();
  }
  
}  // namespace cm730driver
