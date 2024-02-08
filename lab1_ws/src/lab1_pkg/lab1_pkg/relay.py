import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class RelayNode(Node):
    def __init__(self):
        super().__init__('relay')
        self.sub = self.create_subscription(AckermannDriveStamped, 'drive', self.drive_callback, 10) #publishes message type Ackermann over topic "topic"
        self.pub = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)
        self.v = None
        self.d = None

    # get params, publish velocity, v, and steering angle, d, to topic named drive
    def drive_callback(self, msg):
        self.v = msg.drive.speed
        self.d = msg.drive.steering_angle

        msg = AckermannDriveStamped()
        msg.drive.speed = float(self.v*3)
        msg.drive.steering_angle = float(self.d*3)
        self.pub.publish(msg)
        # self.get_logger().info('Publishing velocity and steering angle')


def main(args=None):
    rclpy.init(args=args)

    relay = RelayNode()

    rclpy.spin(relay)
    relay.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
