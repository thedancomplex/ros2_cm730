#ifndef MX_JOINT_CONTROLLER__MX_JOINT_CONTROLLER_HPP_
#define MX_JOINT_CONTROLLER__MX_JOINT_CONTROLLER_HPP_

#include "mx_joint_controller/visibility_control.h"

#include <rclcpp/rclcpp.hpp>
#include <cm730controller_msgs/msg/mx28_info_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>

namespace mx_joint_controller
{

  class MxJointController : public rclcpp::Node
  {
  public:
    MxJointController();
    
    virtual ~MxJointController();

  private:
    using MX28InfoArray = cm730controller_msgs::msg::MX28InfoArray;
    using JointState = sensor_msgs::msg::JointState;
    
    rclcpp::Subscription<MX28InfoArray>::SharedPtr mx28InfoSub_;
    rclcpp::Publisher<JointState>::SharedPtr jointStatePub_;

    double value2Rads(uint16_t value) {
      return (static_cast<int>(value) - 0x0800) / 4096.0 * 2 * M_PI;
    }

  };
  
}  // namespace mx_joint_controller

#endif  // MX_JOINT_CONTROLLER__MX_JOINT_CONTROLLER_HPP_
