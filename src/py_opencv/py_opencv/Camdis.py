import rclpy
from rclpy.node import Node

from my_interfaces.msg import Float                        


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('distance_subscriber')
        self.subscription = self.create_subscription(
            Float,                                               
            'distance',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('I heard Distance: "%.2f" \nangle : "%.2f" \n color_code : "%d"' % (msg.distance,msg.angle,msg.col))  # CHANGE


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
