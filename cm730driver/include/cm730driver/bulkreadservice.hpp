#ifndef CM730DRIVER__BULKREADSERVICE_HPP_
#define CM730DRIVER__BULKREADSERVICE_HPP_

#include "cm730driver/cm730service.hpp"
#include "cm730driver_msgs/srv/bulk_read.hpp"

#define BULK_READ_INSTR 146

namespace cm730driver
{

class BulkReadService : public Cm730Service<BULK_READ_INSTR, cm730driver_msgs::srv::BulkRead,
    BulkReadService>
{
public:
  using Base = Cm730Service<BULK_READ_INSTR, cm730driver_msgs::srv::BulkRead, BulkReadService>;
  using BulkRead = cm730driver_msgs::srv::BulkRead;

  using Base::Base;

  size_t txPacketSize(const BulkRead::Request & request) override
  {
    return HEADER_SIZE + 1 + 3 * request.read_requests.size() / 3 + CHECKSUM_SIZE;
  }

  size_t rxPacketSize(const BulkRead::Request & request) override
  {
    auto total_request_length = 0;
    // Request are 3 tuples: (length, device_id, addr)
    for (auto iter = request.read_requests.begin(); iter != request.read_requests.end();
      std::advance(iter, 3))
    {
      total_request_length += HEADER_SIZE + *iter + CHECKSUM_SIZE;
    }
    return total_request_length;
  }

  uint8_t getDeviceId(const BulkRead::Request & request) override
  {
    (void)request;
    return 254;
  }

  void setDataParameters(const BulkRead::Request & request, Packet & packet) override
  {
    packet[ADDR_PARAMETER] = 0;
    auto cursor = ADDR_PARAMETER + 1;
    for (auto iter = request.read_requests.begin(); iter != request.read_requests.end();
      std::advance(iter, 3))
    {
      packet[cursor++] = *std::next(iter, 0);
      packet[cursor++] = *std::next(iter, 1);
      packet[cursor++] = *std::next(iter, 2);
    }
  }

  void handlePacket(
    Packet const & packet,
    BulkRead::Request const & request,
    BulkRead::Response & response,
    bool timedOut) override
  {
    response.success = !timedOut;
    if (response.success) {
      auto dataCursor = packet.begin();
      // Go through received message, 1 per requested device
      for (auto i = 0u; i < request.read_requests.size() / 3; ++i) {
        auto result = cm730driver_msgs::msg::RangeReadResult{};
        // Get device for which result is, and how much data was sent
        // TODO: check this si the same as requested
        result.device_id = *(dataCursor + ADDR_ID);
        auto length = *(dataCursor + ADDR_LENGTH) - ERROR_SIZE - CHECKSUM_SIZE;
        // Copy response data
        std::advance(dataCursor, ADDR_DATA);
        std::copy(dataCursor, std::next(dataCursor, length), std::back_inserter(result.data));
        // Add to total result
        response.results.push_back(result);
        // Advance to next message
        std::advance(dataCursor, length + CHECKSUM_SIZE);
      }
    }
  }
};

}

#endif  // CM730DRIVER__BULKREADSERVICE_HPP_
