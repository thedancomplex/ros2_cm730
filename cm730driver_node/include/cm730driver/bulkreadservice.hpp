#ifndef CM730DRIVER__BULKREADSERVICE_HPP_
#define CM730DRIVER__BULKREADSERVICE_HPP_

#include "cm730driver/cm730service.hpp"
#include "cm730driver_msgs/srv/bulk_read.hpp"

#define BULK_READ_INSTR 146

namespace cm730driver
{

  class BulkReadService : public Cm730Service<BULK_READ_INSTR, cm730driver_msgs::srv::BulkRead, BulkReadService>
  {
  public:
    using Base = Cm730Service<BULK_READ_INSTR, cm730driver_msgs::srv::BulkRead, BulkReadService>;
    using BulkRead = cm730driver_msgs::srv::BulkRead;
    
    using Base::Base;

    size_t txPacketSize(const BulkRead::Request& request) override {
      (void)request;
      return HEADER_SIZE + 1 + 3 * request.read_requests.size() / 3 + CHECKSUM_SIZE;
    }
    
    size_t rxPacketSize(const BulkRead::Request& request) override {
      return HEADER_SIZE + 1 + CHECKSUM_SIZE;
    }
    
    uint8_t getDeviceId(const BulkRead::Request& request) override
    {
      return request.device_id;
    }
    
    void setDataParameters(const BulkRead::Request& request, Packet& packet) override {
      packet[ADDR_PARAMETER] = 0;
      auto cursor = ADDR_PARAMETER + 1;
      for (auto iter = request.read_requests.begin(); iter != request.read_requests.end();
           std::advance(iter, 3)) {
        packet[cursor++] = *std::next(iter, 0);
        packet[cursor++] = *std::next(iter, 1);
        packet[cursor++] = *std::next(iter, 2);
      }
    }
    
    void handlePacket(Packet const& packet,
                      BulkRead::Response::SharedPtr response,
                      bool timedOut) override
    {
      response->success = !timedOut;
    }
  };
  
}

#endif  // CM730DRIVER__BULKREADSERVICE_HPP_
