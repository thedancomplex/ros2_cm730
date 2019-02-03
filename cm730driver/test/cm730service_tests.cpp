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

TEST(Cm730ServiceTests, calcCheckSum) {
  auto packet = Cm730ServiceBase::Packet{
    0xFF,
    0xFF,
    200,
    2,
    1,
    0,
  };

  ASSERT_EQ(52, Cm730ServiceBase::calcChecksum(packet));
  Cm730ServiceBase::setChecksum(packet);
  ASSERT_EQ(52, packet[5]);
  // Having checksum set should not matter
  ASSERT_EQ(52, Cm730ServiceBase::calcChecksum(packet));
}

TEST(Cm730ServiceTests, checkChecksum) {
  auto packet = Cm730ServiceBase::Packet{
    0xFF,
    0xFF,
    200,
    2,
    1,
    0,
  };

  ASSERT_FALSE(Cm730ServiceBase::checkChecksum(packet));
  Cm730ServiceBase::setChecksum(packet);
  ASSERT_TRUE(Cm730ServiceBase::checkChecksum(packet));
}
