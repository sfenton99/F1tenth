import rclpy
import copy
import numpy as np
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class SafetyNode(Node):
    def __init__(self):
        super().__init__('TTC')
        self.LaserRaw = None
        self.LaserRawTime = None
        self.pose = None 
        self.Vx = 0
        self.dt = None
        self.getAngles = False
        
        self.iTTc1 = [np.Inf]
        self.iTTc2 = [np.Inf]
        self.tol = 2 # break threshold

        self.breakpub = self.create_publisher(AckermannDriveStamped, '/drive', 10) #publishes message type Ackermann over topic "/drive"
        self.lasersub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10) #publishes message type Ackermann over topic "topic"
        self.odomsub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odometry_callback, 10)

  
    def scan_callback(self, msg):
        
        prevLaserRaw = copy.copy(self.LaserRaw)
        self.LaserRaw = msg.ranges
        prevTimestep = copy.copy(self.LaserRawTime)
        self.LaserRawTime = msg.header.stamp.nanosec

        if self.getAngles == False:
            self.angles = [None] * len(self.LaserRaw)
            self.angles[0] = msg.angle_min
            self.angles[-1] = msg.angle_max
            for i in range(len(self.angles)-1):
                self.angles[i+1] = self.angles[i] + msg.angle_increment

        
        if self.LaserRaw is not None and prevLaserRaw is not None:
        # approach 1
            self.dt = (self.LaserRawTime - prevTimestep)*10**-9
            # print(self.dt*10**-9)
            deltar = [(x-y) for x,y in zip(self.LaserRaw,prevLaserRaw)]
            r_dot1 = [max(-dr/self.dt,0) for dr in deltar]
            self.iTTc1 = [np.Inf] * len(r_dot1)
            for i in range(len(r_dot1)):
                if r_dot1[i] < 0:
                    self.iTTc1[i] = -self.LaserRaw[i]/r_dot1[i]


        # approach 2
            r_dot2 = [self.Vx*np.cos(x) for x in self.angles]
            self.iTTc2 = [np.Inf] * len(r_dot2)
            for i in range(len(r_dot2)):
                if r_dot2[i] > 0:
                    self.iTTc2[i] = self.LaserRaw[i]/r_dot2[i]

            

            TTC = min(self.iTTc2)
            # print(TTC)
            # r = min(self.LaserRaw)
            # print(r)
            if TTC is not np.Inf and TTC < self.tol:
                breakmsg = AckermannDriveStamped()
                breakmsg.drive.speed = 0.0
                self.breakpub.publish(breakmsg)
                print("BREAK!!!!!")
                    
      

    def odometry_callback(self, msg):
        self.Vx = msg.twist.twist.linear.x

def main(args=None):
    rclpy.init(args=args)

    TTC = SafetyNode()
    rclpy.spin(TTC)
    TTC.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
