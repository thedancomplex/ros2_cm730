#ifndef CM730DRIVER__SYNCWRITESERVICE_HPP_
#define CM730DRIVER__BULKREADSERVICE_HPP_

#include "cm730driver/cm730service.hpp"
#include "cm730driver_msgs/srv/sync_write.hpp"

#define SYNC_WRITE_INSTR 131

namespace cm730driver
{
class SyncWriteService : public Cm730Service<SYNC_WRITE_INSTR, cm730driver_msgs::srv::SyncWrite,
    SyncWriteService>
{
public:
  using Base = Cm730Service<SYNC_WRITE_INSTR, cm730driver_msgs::srv::SyncWrite, SyncWriteService>;
  using SyncWrite = cm730driver_msgs::srv::SyncWrite;

  using Base::Base;

  size_t txPacketSize(const SyncWrite::Request & request) override
  {
    return HEADER_SIZE + 2 + request.data.size() + CHECKSUM_SIZE;
  }

  size_t rxPacketSize(const SyncWrite::Request & request) override
  {
    (void)request;
    return HEADER_SIZE + CHECKSUM_SIZE;
  }

  uint8_t getDeviceId(const SyncWrite::Request & request) override
  {
    (void)request;
    return 254;
  }

  void setDataParameters(const SyncWrite::Request & request, Packet & packet) override
  {
    packet[ADDR_PARAMETER] = request.address;
    packet[ADDR_PARAMETER + 1] = request.length;

    std::copy(request.data.begin(), request.data.end(), std::next(
        packet.begin(), ADDR_PARAMETER + 2));
  }

  void handlePacket(
    Packet const & packet,
    const SyncWrite::Request & request,
    SyncWrite::Response & response) override
  {
    (void)packet;
    (void)request;
    (void)response;
  }
};
}

#endif
