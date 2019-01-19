#ifndef CM730CONTROLLER__DATAUTIL_HPP_
#define CM730CONTROLLER__DATAUTIL_HPP_

#include "cm730controller/cm730table.hpp"

#include <cstdint>
#include <vector>

namespace cm730controller {

  class DataUtil {
  public:
    static uint8_t getByte(std::vector<uint8_t> const& data, CM730Table addr) {
      return data[uint8_t(addr)];
    }

    static uint16_t getWord(std::vector<uint8_t> const& data, CM730Table addr) {
      return uint16_t{data[uint8_t(addr)]} | (uint16_t{data[uint8_t(addr) + 1]} << 8);
    }
  };

}  // namespace cm730controller

#endif  // CM730CONTROLLER__DATAUTIL_HPP_
