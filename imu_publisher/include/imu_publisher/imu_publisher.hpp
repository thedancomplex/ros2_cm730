// Copyright 2019 Bold Hearts
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef IMU_PUBLISHER__IMU_PUBLISHER_HPP_
#define IMU_PUBLISHER__IMU_PUBLISHER_HPP_

#include "imu_publisher/imu_publisher.hpp"

#include <rclcpp/rclcpp.hpp>
#include <cm730controller_msgs/msg/mx28_info_array.hpp>
#include <cm730controller_msgs/msg/mx28_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>

namespace imu_publisher
{

class IMUPublisher : public rclcpp::Node
{
public:
  IMUPublisher();

  virtual ~IMUPublisher();

private:

};

}  // namespace imu_publisher

#endif  // IMU_PUBLISHER__IMU_PUBLISHER_HPP_
