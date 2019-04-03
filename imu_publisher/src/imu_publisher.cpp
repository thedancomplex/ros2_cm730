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

#include "imu_publisher/imu_publisher.hpp"

namespace imu_publisher
{

IMUPublisher::IMUPublisher()
: rclcpp::Node{"imu_publisher"}
{
    imuStatePub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw");

    cm730InfoSub_ = create_subscription<cm730controller_msgs::msg::CM730Info>(
        "/cm730/cm730info",
        [ = ](cm730controller_msgs::msg::CM730Info::SharedPtr info) {
          auto imuStateMsg = std::make_shared<sensor_msgs::msg::Imu>();

          // CM-730 coordinate systems
          // accelerometer: z down, y forward, x left (right-handed)
          // gyrometer:     z up, y right, x forward (left-handed)
          // (more: http://emanual.robotis.com/docs/en/platform/op/references/)

          // ros2: z up, y left, x forward (right-handed)
          // (more http://www.ros.org/reps/rep-0103.html#coordinate-frame-conventions)

          // accelerometer
          imuStateMsg->linear_acceleration.x =  accelToMS2(info.get()->dyna.accel.at(1)); // y
          imuStateMsg->linear_acceleration.y =  accelToMS2(info.get()->dyna.accel.at(0)); // x
          imuStateMsg->linear_acceleration.z = -accelToMS2(info.get()->dyna.accel.at(2)); // z

          // gyro
          imuStateMsg->angular_velocity.x =  gyroValueToRPS(info.get()->dyna.gyro.at(0)); // x
          imuStateMsg->angular_velocity.y = -gyroValueToRPS(info.get()->dyna.gyro.at(1)); // y
          imuStateMsg->angular_velocity.z =  gyroValueToRPS(info.get()->dyna.gyro.at(2)); // z

          imuStateMsg->header = info.get()->header;
          imuStatePub_->publish(imuStateMsg);
        });

}

IMUPublisher::~IMUPublisher()
{
}

double IMUPublisher::accelToMS2(int value) {

  // Milli Gs per digit
  static auto accelMgsPerDigit = 8.0;
  // Max measurable Gs
  static auto accelGRange = 4.0;
  // Range raw value
  static auto accelDigitalRange = 1024;

  auto gs = (int{value} - accelDigitalRange / 2) * accelMgsPerDigit / 1000;
  auto clampedGs = clamp(gs, -accelGRange, accelGRange);

  return clampedGs * 9.80665;
}

double IMUPublisher::gyroValueToRPS(int value) {

  // Milli degrees per second per digit
  const auto gyroMdpsPerDigit = 448.0;
  // Max measurable degrees per second
  const auto gyroDpsRange = 200.0;
  // Range raw value
  const auto gyroDigitalRange = 1024;

  auto degPerSec = (int{value} - gyroDigitalRange / 2) * gyroMdpsPerDigit / 1000;
  auto clampedDegPerSec = clamp(degPerSec, -gyroDpsRange, gyroDpsRange);

  return clampedDegPerSec / 180 * M_PI;
}

}  // namespace imu_publisher
