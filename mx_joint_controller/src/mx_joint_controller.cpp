#include "mx_joint_controller/mx_joint_controller.hpp"

namespace mx_joint_controller
{

  MxJointController::MxJointController()
    : rclcpp::Node{"mx_joint_controller"}
  {
    auto jointNames = std::vector<std::string>{};
    get_parameter_or("joint_names", jointNames, {
        "base",
        "shoulder-pitch-r",
        "shoulder-pitch-l",
        "shoulder-roll-r",
        "shoulder-roll-l",
        "elbow-r",
        "elbow-l",

        "hip-yaw-r",
        "hip-yaw-l",
        "hip-roll-r",
        "hip-roll-l",
        "hip-pitch-r",
        "hip-pitch-l",
        "knee-r",
        "knee-l",
        "ankle-pitch-r",
        "ankle-pitch-l",
        "ankle-roll-r",
        "ankle-roll-l",

        "head-pan",
        "head-tilt",
      });
    
    jointStatePub_ = create_publisher<JointState>("joint_states");
    
    mx28InfoSub_ = create_subscription<MX28InfoArray>(
      "/cm730/mx28info",
      [=](MX28InfoArray::SharedPtr info) {
        auto jointStateMsg = std::make_shared<JointState>();
        for (auto const& mx : info->mx28s) {
          jointStateMsg->name.push_back(jointNames[mx.stat.id]);
          jointStateMsg->position.push_back(value2Rads(mx.dyna.present_position));
        }
        jointStatePub_->publish(jointStateMsg);
      });
  }
  
  MxJointController::~MxJointController()
  {
  }

}  // namespace mx_joint_controller
