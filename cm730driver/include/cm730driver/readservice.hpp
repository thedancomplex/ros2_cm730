#ifndef CM730DRIVER__READSERVICE_HPP_
#define CM730DRIVER__READSERVICE_HPP_

#include "cm730driver/cm730service.hpp"
#include "cm730driver_msgs/srv/read.hpp"

#define READ_INSTR 2

namespace cm730driver
{

class ReadService : public Cm730Service<READ_INSTR, cm730driver_msgs::srv::Read, ReadService>
{
public:
  using Base = Cm730Service<READ_INSTR, cm730driver_msgs::srv::Read, ReadService>;
  using Read = cm730driver_msgs::srv::Read;

  using Base::Base;

  size_t txPacketSize(const Read::Request & request) override
  {
    (void)request;
    return HEADER_SIZE + 2 + CHECKSUM_SIZE;
  }

  size_t rxPacketSize(const Read::Request & request) override
  {
    return HEADER_SIZE + request.length + CHECKSUM_SIZE;
  }

  uint8_t getDeviceId(const Read::Request & request) override
  {
    return request.device_id;
  }

  void setDataParameters(const Read::Request & request, Packet & packet) override
  {
    packet[ADDR_PARAMETER] = request.address;
    packet[ADDR_PARAMETER + 1] = request.length;
  }

  void handlePacket(
    Packet const & packet,
    Read::Request const & request,
    Read::Response & response,
    bool timedOut) override
  {
    (void)request;
    response.success = !timedOut;
    if (response.success) {
      std::copy(std::next(packet.begin(), HEADER_SIZE), std::prev(packet.end(), CHECKSUM_SIZE),
        std::back_inserter(response.data));
    }
  }
};

}

#endif  // CM730DRIVER__READSERVICE_HPP_
