#
# MOBILE ROBOTS - FI-UNAM, 2027-1
# THE PLATFORM ROS 
#
# Instructions:
# Write a program to avoid obstacles using the laser scan readings
# Required publishers and subscribers are already declared and initialized.
#

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import LaserScan

FULL_NAME = "FULL NAME"

SM_INIT = 0
SM_FORWARD = 10
SM_LEFT = 20
SM_RIGHT = 30
SM_TURN = 40

class RosBasicsNode(Node):
    def __init__(self):
        super().__init__("ros_basics_node")
        self.get_logger().info("INITIALIZING ROS BASICS NODE - " + FULL_NAME)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 1)
        self.pub_point   = self.create_publisher(PointStamped, '/testing_point', 1)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.callback_scan, 1)
        self.rate = self.create_rate(1)
        self.obstacle_front = False
        self.obstacle_left  = False
        self.obstacle_right = False
        return

    def move(self, linear, angular, seconds):
        counter = int(seconds/0.1)
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        while rclpy.ok() and counter > 0:
            self.pub_cmd_vel.publish(msg)
            rclpy.spin_once(self, timeout_sec=0)
            self.get_clock().sleep_for(Duration(seconds=0.1))
            counter -= 1
    
    def spin(self):
        while rclpy.ok():
            #
            # TODO:
            # Use the obstacle variables and the move function to perform the following behavior:
            # If there is an obstable on the left, then turn right
            # If there is an obstacle on the right, then turn left
            # If there is an obstacle in front, then turn around
            #

            #
            # END OF TODO
            #
            rclpy.spin_once(self, timeout_sec=0)
            self.get_clock().sleep_for(Duration(seconds=0.1))
            
    def callback_scan(self, msg):
        #
        # TODO:
        # Do something to detect if there is an obstacle in front of the robot,
        # obstacle on the left, or obstacle on the right. 
        # Set the self 'obstacle_left', 'obstacle_right' and 'obstacle_front' variables
        # with True or False, accordingly.
        # Check online documentation of LaserScan message
        #
        
        return


def main(args=None):
    rclpy.init(args=args)
    ros_basics_node = RosBasicsNode()
    ros_basics_node.spin()
    ros_basics_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
