#include "mx_joint_state_publisher/mx_joint_state_publisher.hpp"

namespace mx_joint_state_publisher
{

  MxJointStatePublisher::MxJointStatePublisher()
    : rclcpp::Node{"mx_joint_state_publisher"}
  {
    jointStatePub_ = create_publisher<JointState>("joint_states");
    
    mx28InfoSub_ = create_subscription<MX28InfoArray>(
      "mx28info",
      [this](MX28InfoArray::SharedPtr info) {
        auto jointStateMsg = std::make_shared<JointState>();
        for (auto const& mx : info->mx28s) {
          jointStateMsg->name.push_back(std::to_string(mx.stat.id));
          jointStateMsg->position.push_back(value2Rads(mx.dyna.present_position));
        }
        jointStatePub_->publish(jointStateMsg);
      });
  }
  
  MxJointStatePublisher::~MxJointStatePublisher()
  {
  }

}  // namespace mx_joint_state_publisher
