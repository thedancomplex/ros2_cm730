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
      auto total_request_length = 0;
      for (auto iter = request.read_requests.begin(); iter != request.read_requests.end();
           std::advance(iter, 3))
        total_request_length += *iter;
      return HEADER_SIZE + total_request_length + CHECKSUM_SIZE;
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
                      BulkRead::Request const& request,
                      BulkRead::Response& response,
                      bool timedOut) override
    {
      response.success = !timedOut;
      if (response.success) {
        auto dataCursor = std::next(packet.begin(), HEADER_SIZE);
        // Go through each requested device
        for (auto iter = request.read_requests.begin(); iter != request.read_requests.end();
             std::advance(iter, 3)) {
          auto result = cm730driver_msgs::msg::RangeReadResult{};
          // Get device and how much was meant to read from original request
          result.device_id = *std::next(iter, 1);
          auto resultLength = *std::next(iter, 0);
          // Copy data into result
          std::copy(dataCursor, std::next(dataCursor, resultLength),
                    std::back_inserter(result.data));
          response.results.push_back(result);
          // Advance location in received packet
          std::advance(dataCursor, resultLength);
        }
      }
    }
  };
  
}

#endif  // CM730DRIVER__BULKREADSERVICE_HPP_
