#include "cm730driver/cm730driver.hpp"

#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>

#include <numeric>

namespace cm730driver
{
  
  Cm730Driver::Cm730Driver()
    : rclcpp::Node{"cm730driver"},
      mDevice{-1}
  {
    open();
    
    mPingServer = create_service<cm730driver_msgs::srv::Ping>(
      "ping",
      [this](std::shared_ptr<cm730driver_msgs::srv::Ping::Request> request,
         std::shared_ptr<cm730driver_msgs::srv::Ping::Response> response)
      {
        auto data = std::array<uint8_t, 6>{
          0xFF, 0xFF,  // prefix
          request->ping.device_id,  // device ID
          2,  // length
          1,  // instruction
          0,  // checksum
        };

        auto sum = std::accumulate(std::next(data.begin(), 2), std::prev(data.end(), 1), 0u);
        data[5] = ~sum;
        write(data.data(), data.size());

        auto nRead = 0;
        while (nRead < data.size())
        {
          auto n = read(data.data() + nRead, data.size() - nRead);
          if (n > 0)
          {
            nRead += n;
            auto str = std::ostringstream{};
            str << "Total read: " << nRead << " - ";
            for (auto i = 0; i < nRead; ++i)
              str << int{data[i]} << " ";
            RCLCPP_INFO(get_logger(), str.str());
          }
        }
        
        response->pong.device_id = data[2];
      });
  }

  Cm730Driver::~Cm730Driver()
  {
    close();
  }

  void Cm730Driver::open()
  {
    auto newtio = termios{};
    auto serinfo = serial_struct{};
    double baudrate = 1000000.0; //bps (1Mbps)

    // Make sure device is closed before trying to open it
    close();

    if ((mDevice = ::open("/dev/ttyUSB0", O_RDWR|O_NOCTTY|O_NDELAY|O_SYNC)) < 0)
    {
      close();
      throw std::runtime_error("Failed opening CM730");
    }

    // Set IO settings
    // You must set 38400bps in order to be able to set non-standard rate!
    newtio.c_cflag      = B38400|CS8|CLOCAL|CREAD;
    newtio.c_iflag      = IGNPAR;
    newtio.c_oflag      = 0;
    newtio.c_lflag      = 0;
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN]   = 0;
    tcsetattr(mDevice, TCSANOW, &newtio);

    // Reset all serial info
    if (ioctl(mDevice, TIOCGSERIAL, &serinfo) < 0)
    {
      close();
      throw std::runtime_error("Failed setting baud rate");
    }

    serinfo.flags &= ~ASYNC_SPD_MASK;
    serinfo.flags |= ASYNC_SPD_CUST;
    serinfo.custom_divisor = serinfo.baud_base / baudrate;

    // Set our serial port to use low latency mode (otherwise the USB
    // driver buffers for 16ms before sending data)
    serinfo.flags |= ASYNC_LOW_LATENCY;
  
    if (ioctl(mDevice, TIOCSSERIAL, &serinfo) < 0)
    {
      close();
      throw std::runtime_error("Failed setting serial flags");
    }

    // FLush all data received but not read
    tcflush(mDevice, TCIFLUSH);

    RCLCPP_INFO(get_logger(), "Successfully opened CM730");
  }

  void Cm730Driver::close()
  {
    if (mDevice > 0)
    {
      if (::close(mDevice) != 0)
      {
        RCLCPP_ERROR(get_logger(), "Failed closing CM730");
        return;
      }
    }
    mDevice = -1;
    RCLCPP_INFO(get_logger(), "Successfully closed CM730");
  }

  int Cm730Driver::write(uint8_t const* outPacket, size_t size)
  {
    auto i = ::write(mDevice, outPacket, size);
    if (i < long(size))
    {
      RCLCPP_ERROR(get_logger(), "Failed writing complete message to CM730");
    }
    else if (i < 0)
    {
      RCLCPP_ERROR(get_logger(), "Failed writing message to CM730");
    }
    return i;
  }

  int Cm730Driver::read(uint8_t* inPacket, size_t size)
  {
    auto i = ::read(mDevice, inPacket, size);
    // If EAGAIN is set, there was no real error, just no data available
    if (i < 0 && errno == EAGAIN)
      i = 0;

    return i;
  }
  
}  // namespace cm730driver
