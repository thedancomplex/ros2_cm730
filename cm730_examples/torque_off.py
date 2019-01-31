import rclpy

from cm730controller_msgs.msg import MX28Command

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('torque_off_publisher')
    publisher = node.create_publisher(MX28Command, '/cm730/mx28command')

    msg = MX28Command(
        device_id=list(range(1, 21)),
        torque=20 * [False]
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
