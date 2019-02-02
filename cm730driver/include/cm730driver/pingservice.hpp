#ifndef CM730DRIVER__PINGSERVICE_HPP_
#define CM730DRIVER__PINGSERVICE_HPP_

#include "cm730driver/cm730service.hpp"
#include "cm730driver_msgs/srv/ping.hpp"

#define PING_INSTR 1

namespace cm730driver
{

class PingService : public Cm730Service<PING_INSTR, cm730driver_msgs::srv::Ping, PingService>
{
public:
  using Base = Cm730Service<PING_INSTR, cm730driver_msgs::srv::Ping, PingService>;
  using Ping = cm730driver_msgs::srv::Ping;

  using Base::Base;

  size_t txPacketSize(const Ping::Request & request) override
  {
    (void)request;
    return HEADER_SIZE + CHECKSUM_SIZE;
  }

  size_t rxPacketSize(const Ping::Request & request) override
  {
    (void)request;
    return HEADER_SIZE + CHECKSUM_SIZE;
  }

  uint8_t getDeviceId(const Ping::Request & request) override
  {
    return request.device_id;
  }

  void setDataParameters(const Ping::Request & request, Packet & packet) override
  {
    (void)request;
    (void)packet;
  }

  void handlePacket(
    Packet const & packet,
    Ping::Request const & request,
    Ping::Response & response,
    bool timedOut) override
  {
    (void)packet;
    (void)request;
    response.success = !timedOut;
  }
};

}

#endif  // CM730DRIVER__PINGSERVICE_HPP_
