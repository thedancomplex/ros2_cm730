#include "cm730driver/cm730driver.hpp"

#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>

namespace cm730driver
{
  
  Cm730Driver::Cm730Driver()
    : rclcpp::Node{"cm730driver"},
      mDevice{-1}
  {
    open();
    
    mPingServer = create_service<cm730driver_msgs::srv::Ping>(
      "ping",
      [](std::shared_ptr<cm730driver_msgs::srv::Ping::Request> request,
         std::shared_ptr<cm730driver_msgs::srv::Ping::Response> response)
      {
        response->pong.device_id = request->ping.device_id;
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

    RCLCPP_INFO(get_logger(), "Successfully opend CM730");
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
  }
  
}  // namespace cm730driver
