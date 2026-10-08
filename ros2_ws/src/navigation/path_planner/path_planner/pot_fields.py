#
# MOBILE ROBOTS - FI-UNAM, 2027-1
# OBSTACLE AVOIDANCE BY POTENTIAL FIELDS
#
# Instructions:
# Complete the code to implement obstacle avoidance by potential fields
# using the attractive and repulsive fields technique.
# Tune the constants alpha and beta to get a smooth movement. 
#

import rclpy
from rclpy.node import Node
#from rclpy.duration import Duration
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseStamped, Point, Vector3
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from builtin_interfaces.msg import Duration
from ament_index_python.packages import get_package_share_directory
import math
import numpy
import time

NAME = "Medrano Solano Enrique"

SM_WAIT_FOR_TF = 0
SM_READY = 10
SM_WAIT_FOR_NEW_GOAL = 20
SM_POT_FIELDS = 40
SM_GOAL_REACHED = 50

class PotFieldsNode(Node):
    def calculate_control(self, goal_x, goal_y, alpha, beta):
        v,w = 0,0
        v_max = 0.5
        w_max = 0.8
        #
        # TODO:
        # Implement the control law given by:
        # v = v_max*math.exp(-error_a*error_a/alpha)
        # w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
        # Set v and w same as simple_move:path_follower
        # Return v and w as a tuble [v,w]
        #
        error_a = math.atan2(goal_y, goal_x)
        error_a = (error_a + math.pi) % (2*math.pi) - math.pi #Se asegura que solo este entre -pi y pi
        v = v_max*math.exp(-error_a*error_a/alpha) #Se aplican las leyes
        w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
        return [v,w]
    
    def attraction_force(self, goal_x, goal_y, eta):
        force_x, force_y = 0,0
        #
        # TODO:
        # Calculate the attraction force, given the robot and goal positions.
        # Return a tuple of the form [force_x, force_y]
        # where force_x and force_y are the X and Y components
        # of the resulting attraction force
        #
        norm = math.sqrt(goal_x**2 + goal_y**2)
        if norm > 0:
            force_x = -eta * (goal_x / norm)
            force_y = -eta * (goal_y / norm)
        
        return numpy.asarray([force_x, force_y])

    def rejection_force(self, laser_readings, zeta, d0):
        N = len(laser_readings)
        if N == 0:
            return [0, 0]
        force_x, force_y = 0, 0
        #
        # TODO:
        # Calculate the total rejection force given by the average
        # of the rejection forces caused by each laser reading.
        # laser_readings is an array where each element is a tuple [distance, angle]
        # both measured w.r.t. robot's frame.
        # See lecture notes for equations to calculate rejection forces.
        # Return a tuple of the form [force_x, force_y]
        # where force_x and force_y are the X and Y components
        # of the resulting rejection force
        #
        for d, theta in laser_readings:
            if 0 < d < d0:
                rho = zeta * math.sqrt(1/d -1/d0)
                force_x += rho * math.cos(theta)
                force_y += rho * math.sin(theta)

        force_x /= N
        force_y /= N
        
        return numpy.asarray([force_x, force_y])
    
    def publish_speed_and_forces(self, v, w, Fa, Fr, F):        
        self.pub_cmd_vel.publish(Twist(linear=Vector3(x=v), angular=Vector3(z=w)))
        mrks = MarkerArray()
        mrks.markers.append(self.get_force_marker(Fa[0], Fa[1], [0.0, 0.0, 1.0, 1.0], 0))
        mrks.markers.append(self.get_force_marker(Fr[0], Fr[1], [1.0, 0.0, 0.0, 1.0], 1))
        mrks.markers.append(self.get_force_marker(F [0], F [1], [0.0, 0.6, 0.0, 1.0], 2))
        self.pub_markers.publish(mrks)

    def get_force_marker(self, force_x, force_y, color, id):
        mrk = Marker()
        mrk.header.frame_id = "base_link"
        mrk.header.stamp = self.get_clock().now().to_msg()
        mrk.lifetime = Duration(sec=1, nanosec=0)
        mrk.ns = "pot_fields"
        mrk.id = id
        mrk.type = Marker.ARROW
        mrk.action = Marker.ADD
        mrk.pose.orientation.w = 1.0
        mrk.color.r, mrk.color.g, mrk.color.b, mrk.color.a = color
        mrk.scale.x, mrk.scale.y, mrk.scale.z = [0.07, 0.1, 0.15]
        mrk.points.append(Point(x=0.0, y=0.0))
        mrk.points.append(Point(x=-force_x, y=-force_y))
        return mrk

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform("map","base_link", rclpy.time.Time())
            robot_x = t.transform.translation.x
            robot_y = t.transform.translation.y
            robot_pose = numpy.asarray([robot_x, robot_y])
            robot_a = math.atan2(t.transform.rotation.z, t.transform.rotation.w)*2
        except TransformException as ex:
            self.get_logger().info("Could not get robot pose")
            robot_pose = numpy.asarray([0.0,0.0])
            robot_a = 0.0
        return robot_pose, robot_a

    def callback_scan(self, msg):
        self.laser_readings = [[msg.ranges[i], msg.angle_min+i*msg.angle_increment] for i in range(len(msg.ranges))]

    def callback_pot_fields_goal(self, msg):
        self.global_goal_x = msg.pose.position.x
        self.global_goal_y = msg.pose.position.y
        self.new_goal_pose = True

    def get_goal_wrt_robot(self):
        Q, theta = self.get_robot_pose()
        delta_x = self.global_goal_x - Q[0]
        delta_y = self.global_goal_y - Q[1]
        goal_x =  delta_x*math.cos(theta) + delta_y*math.sin(theta)
        goal_y = -delta_x*math.sin(theta) + delta_y*math.cos(theta)
        return numpy.asarray([goal_x, goal_y])

    def callback_timer(self):
        Q, theta_r = self.get_robot_pose()
        if self.state == SM_WAIT_FOR_TF:
            self.get_logger().info("Waiting for tf to be ready")
            try:
                t = self.tf_buffer.lookup_transform("map","base_link", rclpy.time.Time())
                self.get_logger().info("Robot pose tf is now available")
                self.state = SM_READY
            except:
                pass
            
        elif self.state == SM_READY:
            self.get_logger().info("Ready to execute new goal pose. Waiting for new goal...")
            self.state = SM_WAIT_FOR_NEW_GOAL

        elif self.state == SM_WAIT_FOR_NEW_GOAL:
            if self.new_goal_pose:
                self.new_goal_pose = False
                Qg = self.get_goal_wrt_robot()
                self.get_logger().info(f"Received new goal point wrt robot: {Qg[0]}, {Qg[1]}")
                self.state = SM_POT_FIELDS

        elif self.state == SM_POT_FIELDS:
            Qg = self.get_goal_wrt_robot()
            if numpy.linalg.norm(Qg) < self.tol:
                self.state = SM_GOAL_REACHED
            else:
                Fa = self.attraction_force(Qg[0], Qg[1], self.eta)
                Fr = self.rejection_force (self.laser_readings, self.zeta, self.d0)
                F = Fa + Fr 
                P = -self.epsilon*F
                [v,w] = self.calculate_control(P[0], P[1], self.alpha, self.beta)
                self.publish_speed_and_forces(v, w, Fa, Fr, F)

        elif self.state == SM_GOAL_REACHED:
            self.pub_cmd_vel.publish(Twist())
            self.get_logger().info("Goal point reached")
            self.state = SM_READY
        
    def __init__(self):
        super().__init__("pot_fields_node")
        self.get_logger().info("INITIALIZING POTENTIAL FIELDS NODE - " + NAME)
        self.new_goal_pose = False
        self.laser_readings = []
        self.global_goal_x = 0.0
        self.global_goal_y = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter('epsilon', 0.5)
        self.declare_parameter('tol', 0.5)
        self.declare_parameter('eta', 1.0)
        self.declare_parameter('zeta',1.0)
        self.declare_parameter('d0',  3.0)
        self.declare_parameter('alpha',0.5)
        self.declare_parameter('beta', 0.5)
        self.epsilon = self.get_parameter('epsilon').get_parameter_value().double_value
        self.tol     = self.get_parameter('tol').get_parameter_value().double_value
        self.eta     = self.get_parameter('eta').get_parameter_value().double_value
        self.zeta    = self.get_parameter('zeta').get_parameter_value().double_value
        self.d0      = self.get_parameter('d0').get_parameter_value().double_value
        self.alpha   = self.get_parameter('alpha').get_parameter_value().double_value
        self.beta    = self.get_parameter('beta').get_parameter_value().double_value
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.callback_scan, 1)
        self.sub_goal = self.create_subscription(PoseStamped, '/goal_pose', self.callback_pot_fields_goal, 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 1)
        self.pub_markers = self.create_publisher(MarkerArray, '/navigation/pot_field_markers', 1)
        self.state = SM_WAIT_FOR_TF
        self.timer = self.create_timer(0.1, self.callback_timer)

def main(args=None):
    rclpy.init(args=args)
    pot_fields_node = PotFieldsNode()
    rclpy.spin(pot_fields_node)
    pot_fields_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
