#include <gtest/gtest.h>

#include "cm730driver/cm730service.hpp"

using namespace cm730driver;

TEST(Cm730ServiceTests, initPacket) {
  auto packet = Cm730ServiceBase::initPacket(10, 200, 2);

  ASSERT_EQ(10u, packet.size());
  ASSERT_EQ(0xFF, packet[0]);
  ASSERT_EQ(0xFF, packet[1]);
  ASSERT_EQ(200, packet[2]);
  ASSERT_EQ(6, packet[3]);
  ASSERT_EQ(2, packet[4]);
}
