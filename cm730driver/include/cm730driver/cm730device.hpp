#ifndef CM730DRIVER__CM730DEVICE_HPP_
#define CM730DRIVER__CM730DEVICE_HPP_

#include <cstdint>
#include <cstring>
#include <string>

namespace cm730driver
{

class Cm730Device
{
public:
  Cm730Device(std::string path);

  void open();
  void close();

  void clear();

  int write(uint8_t const * outPacket, size_t size);
  int read(uint8_t * inPacket, size_t size);

private:
  std::string mPath;
  int mDevice;

};
}

#endif  // CM730DRIVER__CM730DEVICE_HPP_
