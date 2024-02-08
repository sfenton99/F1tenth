import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.v = 0.0
        self.d = 0.0

        self.declare_parameter('v', self.v)
        self.declare_parameter('d', self.d)

        self.pub = self.create_publisher(AckermannDriveStamped, 'drive', 10) #publishes message type Ackermann over topic "topic"
        
        timer_period = 0.01 #how often callback executed
        self.timer = self.create_timer(timer_period, self.timer_callback)


    # get params, publish velocity, v, and steering angle, d, to topic named drive
    def timer_callback(self):

        self.v = self.get_parameter('v').get_parameter_value().double_value
        self.d = self.get_parameter('d').get_parameter_value().double_value

        msg = AckermannDriveStamped()
        msg.drive.speed = float(self.v)
        msg.drive.steering_angle = float(self.d)
        self.pub.publish(msg)
        # self.get_logger().info('Publishing velocity and steering angle')


def main(args=None):
    rclpy.init(args=args)

    talker = TalkerNode()

    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
