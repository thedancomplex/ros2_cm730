import rclpy

from cm730driver_msgs.srv import Ping

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('ping_motors_node')
    ping_client = node.create_client(Ping, '/cm730/ping')

    while not ping_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for CM730 driver to appear...')

    for device_id in range(21):
        req = Ping.Request()
        req.device_id = device_id
        future = ping_client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        e = future.result().error
        print("Device {}:\t{}".format(device_id, "✔" if e == 0 else "✘: {}".format(e)))

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
