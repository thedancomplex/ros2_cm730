# Copyright 2019 Bold Hearts
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy

from mx_joint_controller_msgs.msg import JointCommand

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('head_zero_publisher')
    publisher = node.create_publisher(JointCommand, '/cm730/joint_commands')

    msg = JointCommand(
        name=["head-pan", "head-tilt"],
        p_gain=[4.0, 4.0],
        i_gain=[0.0, 0.0],
        d_gain=[0.0, 0.0],
        position=[0.0, 0.0]
    )

    def timer_callback():
        node.get_logger().info('Publishing: {}'.format(msg))
        publisher.publish(msg)

    timer = node.create_timer(1, timer_callback)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
