#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class FinalProject(Node):
    def __init__(self):
        super().__init__('final_project')

        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('frame_id', 'map')

        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(PoseStamped, goal_topic, 10)

        self.locations = {
            'dinette': (6.59, -1.04),
            'kitchen': (6.68, -3.81),
            'gym': (2.0, 4.26),
            'lunch': (-0.47, 3.0),
            'art': (0.2, -4.0),
            'bedroom': (-6.47, -0.09),
            'desk': (-8.19, 0.91),
            'room': (-6.0, -2.83),
            'living': (0.0, 0.0),
        }

        self.get_logger().info(f'Ready. Publishing goals to {goal_topic} ({self.frame_id})')

    def send_goal(self, x: float, y: float):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0

        self.publisher_.publish(msg)
        self.get_logger().info(f'Goal sent -> x: {x:.2f}, y: {y:.2f}')

    def cli_loop(self):
        print('Locations:', ', '.join(self.locations.keys()))
        while rclpy.ok():
            user_input = input('\nGoal > ').strip().lower()
            if not user_input:
                continue
            if user_input in ('exit', 'quit'):
                break
            if user_input in self.locations:
                x, y = self.locations[user_input]
                self.send_goal(x, y)
                continue
            parts = user_input.replace(',', ' ').split()
            if len(parts) == 2:
                try:
                    self.send_goal(float(parts[0]), float(parts[1]))
                except ValueError:
                    print('Invalid coordinates.')
            else:
                print('Invalid input.')


def main(args=None):
    rclpy.init(args=args)
    node = FinalProject()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    node.cli_loop()
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()