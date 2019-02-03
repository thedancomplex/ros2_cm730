#include <gtest/gtest.h>

#include "mx_joint_controller/mx_joint_controller.hpp"

using namespace mx_joint_controller;

TEST(MxJointControllerTests, value2Rads) {
  ASSERT_EQ(0.0, MxJointController::value2Rads(0x800));
  ASSERT_EQ(-M_PI, MxJointController::value2Rads(0));
  ASSERT_EQ(M_PI, MxJointController::value2Rads(0x1000));
}

TEST(MxJointControllerTests, rads2Value) {
  ASSERT_EQ(0x800, MxJointController::rads2Value(0.0));
  ASSERT_EQ(0, MxJointController::rads2Value(-M_PI));
  ASSERT_EQ(0x1000, MxJointController::rads2Value(M_PI));
}
