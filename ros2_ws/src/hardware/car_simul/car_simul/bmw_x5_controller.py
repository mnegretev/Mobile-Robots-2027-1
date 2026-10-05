#!/usr/bin/env python3

import rclpy

from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from scipy.spatial.transform import Rotation


class CarDriver:

    def init(self, webots_node, properties):

        self.robot = webots_node.robot

        rclpy.init(args=None)

        self.node = rclpy.create_node('car_driver_node')

        self.cmd_sub = self.node.create_subscription(AckermannDrive, '/cmd_ackermann', self.cmd_callback, 1)

        self.tf_broadcaster = TransformBroadcaster(self.node)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self.node)

        self.car_node = self.robot.getSelf()

        self.publish_lidar_tf()

        self.robot.setCruisingSpeed(0.0)
        self.robot.setSteeringAngle(0.0)

        self.node.get_logger().info('BMW X5 driver ready....')


    def publish_lidar_tf(self):

        t = TransformStamped()

        t.header.stamp = self.node.get_clock().now().to_msg()

        t.header.frame_id = 'base_link'
        t.child_frame_id = 'lidar_link'

        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.65

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(t)


    def cmd_callback(self, msg):

        self.robot.setCruisingSpeed(msg.speed)
        self.robot.setSteeringAngle(msg.steering_angle)

        self.node.get_logger().info(
            f'speed={msg.speed:.2f} m/s '
            f'({msg.speed:.2f} km/h), '
            f'steering={msg.steering_angle:.3f} rad'
        )

    def publish_car_tf(self):

        position = self.car_node.getPosition()
        orientation = self.car_node.getOrientation()

        matrix = [
            orientation[0:3],
            orientation[3:6],
            orientation[6:9]
        ]

        qx, qy, qz, qw = Rotation.from_matrix(matrix).as_quat()

        t = TransformStamped()

        sim_time = self.robot.getTime()

        t.header.stamp.sec = int(sim_time)
        t.header.stamp.nanosec = int(
            (sim_time - int(sim_time)) * 1e9
        )

        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = float(position[0])
        t.transform.translation.y = float(position[1])
        t.transform.translation.z = float(position[2])

        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(t)


    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        self.publish_car_tf()