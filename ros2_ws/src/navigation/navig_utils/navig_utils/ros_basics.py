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

FULL_NAME = "SOLORIO GONZALEZ ALDO BRUNO"

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
            #move(Velocidad, Velocidad Angular, Tiempo [s])
            if self.obstacle_left:
                self.get_logger().info("Obstacle on the left")
                self.move(0, -0.48, 0.50)
            elif self.obstacle_right:
                self.get_logger().info("Obstacle on the right")
                self.move(0, 0.48, 0.50)
            elif self.obstacle_front:
                self.get_logger().info("Obstacle in front")
                self.move(0, 0.48, 1.0)
            else:
                self.get_logger().info("Moving forward")
                self.move(0.2, 0.0, 0.2)
            #
            # END OF TODO
            #
            rclpy.spin_once(self, timeout_sec=0)
            self.get_clock().sleep_for(Duration(seconds=0.1))
    
    #Funcion propuesta
    def get_representative_distance(self, ranges, range_min, range_max):
        valid_ranges = sorted(
            distance
            for distance in ranges
            if range_min <= distance <= range_max
        )

        if len(valid_ranges) == 0:
            return float('inf')

        five_smallest = valid_ranges[:5]

        if max(five_smallest) - min(five_smallest) > 0.5:
            return min(five_smallest)

        return sum(five_smallest) / len(five_smallest)

    def callback_scan(self, msg):

        #
        # TODO:
        # Do something to detect if there is an obstacle in front of the robot,
        # obstacle on the left, or obstacle on the right. 
        # Set the self 'obstacle_left', 'obstacle_right' and 'obstacle_front' variables
        # with True or False, accordingly.
        # Check online documentation of LaserScan message
        #

        n = len(msg.ranges)

        section = n // 8
        half_section = section // 2
        center = n // 2

        front_start = center - half_section
        front_end = front_start + section

        left_start = front_end + section
        left_end = left_start + section

        right_end = front_start - section
        right_start = right_end - section

        front_ranges = msg.ranges[front_start:front_end]
        left_ranges = msg.ranges[left_start:left_end]
        right_ranges = msg.ranges[right_start:right_end]

        front_distance = self.get_representative_distance(
            front_ranges,
            msg.range_min,
            msg.range_max
        )

        left_distance = self.get_representative_distance(
            left_ranges,
            msg.range_min,
            msg.range_max
        )

        right_distance = self.get_representative_distance(
            right_ranges,
            msg.range_min,
            msg.range_max
        )

        self.obstacle_front = front_distance < 0.7
        self.obstacle_left = left_distance < 0.7
        self.obstacle_right = right_distance < 0.7

        self.get_logger().info(
            f"Distances: front={front_distance:.2f}, "
            f"left={left_distance:.2f}, "
            f"right={right_distance:.2f}"
        )

        return


def main(args=None):
    rclpy.init(args=args)
    ros_basics_node = RosBasicsNode()
    ros_basics_node.spin()
    ros_basics_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
