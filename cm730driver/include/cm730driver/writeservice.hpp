#ifndef CM730DRIVER__WRITESERVICE_HPP_
#define CM730DRIVER__WRITESERVICE_HPP_

#include "cm730driver/cm730service.hpp"
#include "cm730driver_msgs/srv/write.hpp"

#define WRITE_INSTR 3

namespace cm730driver
{

class WriteService : public Cm730Service<WRITE_INSTR, cm730driver_msgs::srv::Write, WriteService>
{
public:
  using Base = Cm730Service<WRITE_INSTR, cm730driver_msgs::srv::Write, WriteService>;
  using Write = cm730driver_msgs::srv::Write;

  using Base::Base;

  size_t txPacketSize(const Write::Request & request) override
  {
    return HEADER_SIZE + 1 + request.data.size() + CHECKSUM_SIZE;
  }

  size_t rxPacketSize(const Write::Request & request) override
  {
    (void)request;
    return HEADER_SIZE + CHECKSUM_SIZE;
  }

  uint8_t getDeviceId(const Write::Request & request) override
  {
    return request.device_id;
  }

  void setDataParameters(const Write::Request & request, Packet & packet) override
  {
    packet[ADDR_PARAMETER] = request.address;
    std::copy(request.data.begin(), request.data.end(),
      std::next(packet.begin(), ADDR_PARAMETER + 1));
  }

  void handlePacket(
    Packet const & packet,
    Write::Request const & request,
    Write::Response & response,
    bool timedOut) override
  {
    (void)packet;
    (void)request;
    response.success = !timedOut;
  }
};

}

#endif  // CM730DRIVER__WRITESERVICE_HPP_
