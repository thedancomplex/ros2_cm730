#ifndef MX_JOINT_CONTROLLER__MX_JOINT_CONTROLLER_HPP_
#define MX_JOINT_CONTROLLER__MX_JOINT_CONTROLLER_HPP_

#include "mx_joint_controller/visibility_control.h"

#include <rclcpp/rclcpp.hpp>
#include <cm730controller_msgs/msg/mx28_info_array.hpp>
#include <cm730controller_msgs/msg/mx28_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mx_joint_controller_msgs/msg/joint_command.hpp>
#include <cmath>

namespace mx_joint_controller
{

class MxJointController : public rclcpp::Node
{
public:
  MxJointController();

  virtual ~MxJointController();

  static double value2Rads(uint16_t value)
  {
    return (static_cast<int>(value) - 0x0800) * (2 * M_PI) / 4096.0;
  }

  static uint16_t rads2Value(float rads)
  {
    return std::round(rads * 4096.0 / (2 * M_PI) ) + 0x800;
  }

private:
  using MX28InfoArray = cm730controller_msgs::msg::MX28InfoArray;
  using MX28Command = cm730controller_msgs::msg::MX28Command;

  using JointState = sensor_msgs::msg::JointState;
  using JointCommand = mx_joint_controller_msgs::msg::JointCommand;

  rclcpp::Subscription<MX28InfoArray>::SharedPtr mx28InfoSub_;
  rclcpp::Publisher<MX28Command>::SharedPtr mx28CommandPub_;

  rclcpp::Publisher<JointState>::SharedPtr jointStatePub_;
  rclcpp::Subscription<JointCommand>::SharedPtr jointCommandSub_;

};

}  // namespace mx_joint_controller

#endif  // MX_JOINT_CONTROLLER__MX_JOINT_CONTROLLER_HPP_
