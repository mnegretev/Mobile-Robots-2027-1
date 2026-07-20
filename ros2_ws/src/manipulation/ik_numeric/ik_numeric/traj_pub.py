#
# MOBILE ROBOTS - FI-UNAM, 2026-2
# Node for testing joint trajectories
#

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class TrajectoryPubNode(Node):
    def __init__(self):
        super().__init__("trajectory_pub_node")
        self.get_logger().info("INITIALIZING TRAJECTORY PUBLISHER NODE ...")
        self.declare_parameter('Q', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.pub_traj = self.create_publisher(JointTrajectory, '/xarm6_traj_controller/joint_trajectory', 1)

    def spin(self):
        counter = 10
        while rclpy.ok() and counter > 0:
            rclpy.spin_once(self, timeout_sec=0)
            time.sleep(0.1)
            counter -= 1
        Q = self.get_parameter('Q').get_parameter_value().double_array_value
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        p = JointTrajectoryPoint()
        p.positions = [Q[0], Q[1], Q[2], Q[3], Q[4], Q[5]]
        p.time_from_start.sec = 1
        msg.points.append(p)
        self.get_logger().info("Sending trajectory:  " + str(msg))
        self.pub_traj.publish(msg)
        rclpy.spin_once(self, timeout_sec=0)
        rclpy.spin_once(self, timeout_sec=0)
            

def main(args=None):
    rclpy.init(args=args)
    traj_pub_node = TrajectoryPubNode()
    traj_pub_node.spin()
    traj_pub_node.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
