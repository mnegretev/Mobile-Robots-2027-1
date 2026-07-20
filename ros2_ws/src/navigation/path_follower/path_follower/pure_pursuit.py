#
# MOBILE ROBOTS - FI-UNAM, 2026-2
# PATH FOLLOWING BY PURE PURSUIT
#
# Instructions:
# Write the code necessary to move the robot along a given path.
# Consider a differential base. Max linear and angular speeds
# must be 0.8 and 1.0 respectively.
#

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from navig_msgs.srv import ProcessPath
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from ament_index_python.packages import get_package_share_directory
import math
import numpy

NAME = "FULL NAME"

SM_INIT = 0
SM_WAIT_FOR_NEW_GOAL = 10
SM_PLAN_PATH = 20
SM_SMOOTH_PATH = 30
SM_FOLLOWING_PATH = 40
SM_SAVE_DATA = 50

class PurePursuitNode(Node):
    def calculate_control(self, robot_x, robot_y, robot_a, goal_x, goal_y, alpha, beta, v_max, w_max):
        v,w = 0,0
        #
        # TODO:
        # Implement the control law given by:
        #
        error_a = math.atan2(goal_y - robot_y, goal_x - robot_x) - robot_a
        error_a = (error_a + math.pi)%(2*math.pi) - math.pi
        v = v_max*math.exp(-error_a*error_a/alpha)
        w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
        #
        # where error_a is the angle error
        # and v_max, w_max, alpha and beta, are tunning constants.
        # Remember to keep error angle in the interval (-pi,pi]
        # Return the tuple [v,w]
        #
                
        return [v,w]

    def pure_pursuit(self, path, alpha, beta, v_max, w_max, tol):
        #
        # TODO:
        # Use the calculate_control function to move the robot along the path.
        # Path is given as a sequence of points [[x0,y0], [x1,y1], ..., [xn,yn]]
        # You can use the following steps to perform the path tracking by pure pursuit:
        #
        # Set goal point as the first point of the path
        # Get robot position with Pr, robot_a = get_robot_pose()
        #
        # WHILE distance to last point > tol and rclpy.ok():
        #     Calculate control signals v and w
        #     Publish the control signals with the function publish_and_save_data()
        #     Get robot position
        #     If dist to goal point is less than 0.3 (you can change this constant)
        #         Change goal point to the next point in the path
        #
        idx = 0
        Pg = path[idx]
        Pr, robot_a = self.get_robot_pose()
        while numpy.linalg.norm(path[-1] - Pr) > tol and rclpy.ok():
            v,w = self.calculate_control(Pr[0],Pr[1],robot_a,Pg[0],Pg[1],alpha,beta,v_max,w_max)
            self.publish_and_save_data(Pr[0],Pr[1], robot_a, Pg[0],Pg[1], v,w)
            Pr, robot_a = self.get_robot_pose()
            if numpy.linalg.norm(Pg - Pr) < 0.3:
                idx = min(len(path)-1, idx + 1)
                Pg = path[idx]
        #
        # END OF WHILE
        #
        return

    def publish_and_save_data(self, robot_x, robot_y, robot_a, goal_x, goal_y, v,w):
        self.nav_data.append([robot_x, robot_y, robot_a, goal_x, goal_y, v, w])
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub_cmd_vel.publish(msg)
        rclpy.spin_once(self)
        self.get_clock().sleep_for(Duration(seconds=0.005))

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform("map","base_link", rclpy.time.Time())
            robot_x = t.transform.translation.x
            robot_y = t.transform.translation.y
            robot_pose = numpy.asarray([robot_x, robot_y])
            robot_a = math.atan2(t.transform.rotation.z, t.transform.rotation.w)*2
            self.robot_pose = robot_pose
            self.robot_a = robot_a
        except TransformException as ex:
            self.get_logger().info("Could not get robot pose")
            robot_pose = numpy.asarray([0.0,0.0])
            robot_a = 0.0
        return robot_pose, robot_a

    def callback_goal_pose(self, msg):
        self.goal_pose = numpy.asarray([msg.pose.position.x, msg.pose.position.y])
        self.get_logger().info("Received new goal pose: " + str(self.goal_pose))
        self.new_goal_pose = True

    def __init__(self):
        super().__init__("pure_pursuit_node")
        self.get_logger().info("INITIALIZING PATH FOLLOWER BY PURE PURSUIT NODE ...")
        self.nav_data = []
        self.data_file = get_package_share_directory('path_follower') + "/data.txt" 
        self.robot_pose = numpy.asarray([0.0,0.0])
        self.robot_a = 0.0
        self.new_goal_pose = False
        self.goal_pose = numpy.asarray([0.0,0.0])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter('v_max', 0.5)
        self.declare_parameter('w_max', 0.5)
        self.declare_parameter('alpha', 1.0)
        self.declare_parameter('beta',  1.0)
        self.declare_parameter('tol',  0.3)
        self.clt_plan_path = self.create_client(GetPlan, '/path_planning/plan_path')
        self.clt_smooth_path = self.create_client(ProcessPath, '/path_planning/smooth_path')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 1)
        self.pub_goal_reached = self.create_publisher(Bool, '/navigation/goal_reached', 1)
        self.sub_goal_pose = self.create_subscription(PoseStamped, '/goal_pose', self.callback_goal_pose, 1)

    def spin(self):
        robot_pose_tf_ready = False
        self.get_logger().info("Waiting for plan path service...")
        while not self.clt_plan_path.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for plan path service...')
        self.get_logger().info("Plan path service is now available...")
        clt_timeout = 3
        self.get_logger().info("Waiting for smooth path service...")
        while not self.clt_smooth_path.wait_for_service(timeout_sec=0.5) and clt_timeout > 0:
            self.get_logger().info("Waiting for smooth path service...")
            clt_timeout -= 1
        if self.clt_smooth_path.wait_for_service(timeout_sec=0.5):
            self.get_logger().info("Smooth path service is available")
        else:
            self.get_logger().info("Smooth path service is not available")
        self.get_logger().info("Waiting for robot pose tf to be available")
        while rclpy.ok() and not robot_pose_tf_ready:
            try:
                t = self.tf_buffer.lookup_transform("map","base_link", rclpy.time.Time())
                robot_pose_tf_ready = True
            except TransformException as ex:
                robot_pose_tf_ready = False
            rclpy.spin_once(self)
            self.get_clock().sleep_for(Duration(seconds=0.02))
        self.get_logger().info("Robot pose tf is now available")

        state = SM_INIT
        while rclpy.ok():
            robot_p, robot_a = self.get_robot_pose()
            if state == SM_INIT:
                self.get_logger().info("Ready to execute new path. Waiting for new goal...")
                state = SM_WAIT_FOR_NEW_GOAL

            elif state == SM_WAIT_FOR_NEW_GOAL:
                if self.new_goal_pose:
                    self.new_goal_pose = False
                    state = SM_PLAN_PATH

            elif state == SM_PLAN_PATH:
                self.get_logger().info("Trying to plan path from" + str(self.robot_pose) + " to "+ str(self.goal_pose))
                request = GetPlan.Request()
                request.start.pose.position.x = self.robot_pose[0]
                request.start.pose.position.y = self.robot_pose[1]
                request.goal.pose.position.x = self.goal_pose[0]
                request.goal.pose.position.y = self.goal_pose[1]
                future = self.clt_plan_path.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                path = future.result().plan
                self.get_logger().info("Path planned with " + str(len(path.poses)) + " points")
                state = SM_SMOOTH_PATH

            elif state == SM_SMOOTH_PATH:
                req = ProcessPath.Request()
                req.path = path
                if self.clt_smooth_path.wait_for_service(timeout_sec=0.1):
                    future = self.clt_smooth_path.call_async(req)
                    rclpy.spin_until_future_complete(self, future)
                    path = future.result().processed_path
                    self.get_logger().info("Path smoothed succesfully")
                else:
                    self.get_logger().info("Smooth path service is not available")
                state = SM_FOLLOWING_PATH

            elif state == SM_FOLLOWING_PATH:
                v_max = self.get_parameter('v_max').get_parameter_value().double_value
                w_max = self.get_parameter('w_max').get_parameter_value().double_value
                alpha = self.get_parameter('alpha').get_parameter_value().double_value
                beta  = self.get_parameter('beta').get_parameter_value().double_value
                tol   = self.get_parameter('tol').get_parameter_value().double_value
                self.get_logger().info("Following path with [v_max, w_max, alpha, beta, tol]="+str([v_max, w_max, alpha, beta, tol]))
                path_points = [numpy.asarray([p.pose.position.x, p.pose.position.y]) for p in path.poses]
                self.pure_pursuit(path_points, alpha, beta, v_max, w_max, tol)
                self.pub_cmd_vel.publish(Twist())
                self.pub_goal_reached.publish(Bool(data=True))
                self.get_logger().info("Global goal point reached")
                state = SM_SAVE_DATA

            elif state == SM_SAVE_DATA:
                s = ""
                for d in self.nav_data:
                    s += str(d[0]) +","+ str(d[1]) +","+ str(d[2]) +","+ str(d[3]) +","+ str(d[4]) +","+ str(d[5]) +","+ str(d[6]) + "\n"
                f = open(self.data_file, "w")
                f.write(s)
                f.close()
                state = SM_INIT
                
            rclpy.spin_once(self)
            self.get_clock().sleep_for(Duration(seconds=0.005))


def main(args=None):
    rclpy.init(args=args)
    pure_pursuit_node = PurePursuitNode()
    pure_pursuit_node.spin()
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

