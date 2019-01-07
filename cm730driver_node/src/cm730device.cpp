#include "cm730driver/cm730device.hpp"

#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>

#include <stdexcept>
#include <rclcpp/logging.hpp>

using namespace cm730driver;

Cm730Device::Cm730Device(std::string path)
  : mPath{std::move(path)}
{
}

void Cm730Device::open()
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

  RCLCPP_INFO(rclcpp::get_logger("cm730device"), "Successfully opened CM730");
}

void Cm730Device::close()
{
  if (mDevice > 0)
  {
    if (::close(mDevice) != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("cm730device"), "Failed closing CM730");
      return;
    }
  }
  mDevice = -1;
  RCLCPP_INFO(rclcpp::get_logger("cm730device"), "Successfully closed CM730");
}

void Cm730Device::clear()
{
  tcflush(mDevice, TCIFLUSH);
}
  
int Cm730Device::write(uint8_t const* outPacket, size_t size)
{
  auto i = ::write(mDevice, outPacket, size);
  if (i < long(size))
  {
    RCLCPP_ERROR(rclcpp::get_logger("cm730device"), "Failed writing complete message to CM730");
  }
  else if (i < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("cm730device"), "Failed writing message to CM730");
  }
  return i;
}

int Cm730Device::read(uint8_t* inPacket, size_t size)
{
  auto i = ::read(mDevice, inPacket, size);
  // If EAGAIN is set, there was no real error, just no data available
  if (i < 0 && errno == EAGAIN)
    i = 0;

  return i;
}
