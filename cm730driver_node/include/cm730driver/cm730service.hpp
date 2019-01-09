#ifndef CM730DRIVER__CM730SERVICE_HPP_
#define CM730DRIVER__CM730SERVICE_HPP_

#include "cm730driver/cm730device.hpp"

#include <array>
#include <vector>
#include <numeric>
#include <memory>
#include <sstream>

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

namespace cm730driver
{
  class Cm730Device;
  
  /**CM730 Service base class
   *
   * Provides standard methods for any service that transmits and
   * receives messages to and from a CM730.
   */
  template<uint8_t INSTR, class ServiceT, class Derived>
  class Cm730Service
  {
  public:
    static constexpr uint8_t HEADER_SIZE = 5;
    static constexpr uint8_t CHECKSUM_SIZE = 1;
    
    /// Indexes to parts in CM730 packets
    enum  PacketAddr : uint8_t {
      ADDR_ID = 2,
      ADDR_LENGTH = 3,
      ADDR_INSTRUCTION = 4,
      ADDR_PARAMETER = 5,  // For TX packet
      ADDR_ERROR = 5       // For RX packet
    };

    Cm730Service(std::shared_ptr<Cm730Device> device, std::shared_ptr<rclcpp::Clock> clock);
    
    /// Fixed size packet data
    using Packet = std::vector<uint8_t>;

    void handle(std::shared_ptr<typename ServiceT::Request> request,
                std::shared_ptr<typename ServiceT::Response> response);
    
    /** Size of packets to send to CM730
     *
     * Must include header and checksum
     */
    virtual size_t txPacketSize() = 0;

    /** Size of packets received from CM730
     *
     * Must include header and checksum
     */
    virtual size_t rxPacketSize(const typename ServiceT::Request& request) = 0;

    /** Device ID for packet
     *
     * @param request Service request that can be used to determine device ID
     */
    virtual uint8_t getDeviceId(const typename ServiceT::Request& request) = 0;
    
    /** Set parameter bytes
     *
     * @param request Service request that can be used to determine parameters
     * @param packet Pakcet to send, with header initialised
     */
    virtual void setDataParameters(const typename ServiceT::Request& request, Packet& packet) = 0;

    virtual void handlePacket(Packet const& packet,
                              std::shared_ptr<typename ServiceT::Response> response,
                              bool timedOut) = 0;

    static std::tuple<std::shared_ptr<Derived>, typename rclcpp::Service<ServiceT>::SharedPtr>
    create(
      rclcpp::Node& node, std::string const& serviceName,
      std::shared_ptr<Cm730Device> device, std::shared_ptr<rclcpp::Clock> clock)
    {
      auto cm730Service = std::make_shared<Derived>(std::move(device), std::move(clock));
      auto rclcppService = node.create_service<ServiceT>(
        serviceName,
        [=](std::shared_ptr<typename ServiceT::Request> request,
            std::shared_ptr<typename ServiceT::Response> response) {
          cm730Service->handle(request, response);
        });

      return std::make_tuple(cm730Service, rclcppService);
    }
    
  private:
    /// Prepare packet header data
    Packet initPacket(size_t size, uint8_t deviceId) const;

    /// Set checksum of a prepared packet
    void setChecksum(Packet& packet) const;

    /// Calculate checksum of a prepared packet
    uint8_t calcChecksum(Packet const& packet) const;

    std::shared_ptr<Cm730Device> mDevice;
    std::shared_ptr<rclcpp::Clock> mClock;
};



  template<uint8_t INSTR, class ServiceT, class Derived>
  Cm730Service<INSTR, ServiceT, Derived>::Cm730Service(std::shared_ptr<Cm730Device> device, std::shared_ptr<rclcpp::Clock> clock)
    : mDevice{std::move(device)},
      mClock{std::move(clock)}
  {}
  
  template<uint8_t INSTR, class ServiceT, class Derived>
  void Cm730Service<INSTR, ServiceT, Derived>::handle(
    std::shared_ptr<typename ServiceT::Request> request,
    std::shared_ptr<typename ServiceT::Response> response)
  {
    // Flush any unread bytes
    mDevice->clear();

    // Prepare packet to send
    auto txPacket = initPacket(txPacketSize(), getDeviceId(*request));
    setDataParameters(*request, txPacket);
    setChecksum(txPacket);

    auto str = std::ostringstream{};
    str << "Writing: ";
    for (auto i = 0; i < txPacket.size(); ++i)
      str << int{txPacket[i]} << " ";
    RCLCPP_INFO(rclcpp::get_logger("cm730service"), str.str());
    
    // Send packet
    mDevice->write(txPacket.data(), txPacket.size());

    // Prepare for reading response
    auto rxPacket = Packet(rxPacketSize(*request));
    
    auto readStartTime = mClock->now();
    auto nRead = 0u;
    while (nRead < rxPacket.size())
    {
      // Check if we have timed out and give up
      auto readingTime = mClock->now() - readStartTime;
      if (readingTime.nanoseconds() / 1e6 > 12)
      {
        RCLCPP_INFO(rclcpp::get_logger("cm730service"), "Timed out");
        handlePacket(rxPacket, response, true);
        return;
      }

      // Read as many bytes as are available, up to how many are still missing
      auto n = mDevice->read(rxPacket.data() + nRead, rxPacket.size() - nRead);
      if (n > 0)
      {
        // A positive amount of bytes is good
        // TODO: handle negative, which indicates an error
        nRead += n;

        // Log what we've read so far
        // TODO: use debug level
        auto str = std::ostringstream{};
        str << "Total read: " << nRead << " - ";
        for (auto i = 0; i < nRead; ++i)
          str << int{rxPacket[i]} << " ";
        RCLCPP_INFO(rclcpp::get_logger("cm730service"), str.str());
      }
      
      rclcpp::sleep_for(std::chrono::microseconds{100});
    }

    // Successfully read full packet, create response
    handlePacket(rxPacket, response, false);
  }
  
  template<uint8_t INSTR, class ServiceT, class Derived>
  typename Cm730Service<INSTR, ServiceT, Derived>::Packet Cm730Service<INSTR, ServiceT, Derived>::initPacket(size_t size, uint8_t deviceId) const
  {
    auto data = Packet(size);
    data[0] = data[1] = 0xFF;
    data[ADDR_ID] = deviceId;
    data[ADDR_LENGTH] = size - (uint8_t)ADDR_INSTRUCTION;    
    data[ADDR_INSTRUCTION] = INSTR;
    return data;
  }

  template<uint8_t INSTR, class ServiceT, class Derived>
  void Cm730Service<INSTR, ServiceT, Derived>::setChecksum(Packet& data) const
  {
    *std::prev(data.end(), 1) = calcChecksum(data);
  }

  template<uint8_t INSTR, class ServiceT, class Derived>
  uint8_t Cm730Service<INSTR, ServiceT, Derived>::calcChecksum(Packet const& data) const
  {
    auto sum = std::accumulate(std::next(data.begin(), 2), std::prev(data.end(), 1), 0u);
    return ~sum;
  }
}

#endif  // CM730DRIVER__CM730SERVICE_HPP_
