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
