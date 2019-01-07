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

    using Base::Base;

    size_t txPacketSize() override { return 6; }
    
    size_t rxPacketSize() override { return 6; }
    
    uint8_t getDeviceId(const cm730driver_msgs::srv::Ping::Request& request) override
    {
      return request.ping.device_id;
    }
    
    void setDataParameters(const cm730driver_msgs::srv::Ping::Request& request, Packet& packet) override {}
    
    void handlePacket(Packet const& packet,
                      cm730driver_msgs::srv::Ping::Response::SharedPtr response,
                      bool timedOut) override
    {
      response->pong.success = !timedOut;
    }
  };
  
}

#endif  // CM730DRIVER__PINGSERVICE_HPP_
