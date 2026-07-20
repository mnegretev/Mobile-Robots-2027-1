#
# MOBILE ROBOTS - FI-UNAM, 2026-2
# PATH FOLLOWING BY STANLEY CONTROLLER
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

class StanleyNode(Node):
    def calculate_control(self, robot_x, robot_y, robot_a, x_i, y_i, theta_i, Kd, Ka, v_max, w_max):
        v,w = 0,0
        Kv = 5.0
        #
        # TODO:
        # Implement the Stanley controller given by:
        #
        # theta_e = error angle between theta_i and the vector from point (xi,yi) to robot position
        theta_e = (theta_i - math.atan2(robot_y-y_i, robot_x-x_i) + math.pi)%(math.pi*2) - math.pi
        # et = signed distance from point (xi,yi) to robot position
        et = math.sqrt((robot_x - x_i)**2 + (robot_y - y_i)**2)*numpy.sign(theta_e)       
        # alpha = (theta_ i - robot_a ) remeber to keep angle in (-pi,pi]
        alpha = (theta_i - robot_a + math.pi)%(math.pi*2) - math.pi
        # v = v_max*e^(-Kv*(et^2+alpha^2))
        v = v_max*math.exp(-Kv*(et**2 + alpha**2))
        # w = Ka*alpha + Kd*et
        w = Ka*alpha + Kd*et
        w = max(-w_max, min(w_max, w))
        # Remember to keep w in (-w_max,w_max)
        # Return the tuple [v,w]
        #
        
        return [v,w]

    def get_nearest_point_and_angle(self, path, robot_x, robot_y):
        xi, yi = 0,0
        Pr = numpy.asarray([robot_x, robot_y])
        distances = [numpy.linalg.norm(Pr - p) for p in path]
        i = numpy.argmin(distances)
        i_next = min(i+1, len(path)-1)
        i_prev = max(i-1, 0)
        Pi = path[i]
        Pn = path[i_next]
        Pp = path[i_prev]
        theta_i = math.atan2(Pn[1] - Pp[1], Pn[0] - Pp[0])
        return Pi[0], Pi[1], theta_i

    def stanley_path_following(self, path, Kd, Ka, v_max, w_max, tol):
        #
        # TODO:
        # Use the calculate_control function to move the robot along the path.
        # Path is given as a sequence of points [[x0,y0], [x1,y1], ..., [xn,yn]]
        # You can use the following steps to perform the path tracking by pure pursuit:
        #
        # Get robot position with Pr, robot_a = get_robot_pose()
        # WHILE distance to last point > tol and rclpy.ok():
        #     Get nearest point (xi,yi) and angle theta_i by calling the corresponding function
        #     Calculate control signals v and w
        #     Publish the control signals with the function publish_and_save_data()
        #     Get robot position
        #
        Pr, robot_a = self.get_robot_pose()
        while numpy.linalg.norm(path[-1] - Pr)>tol and rclpy.ok():
            xi, yi, theta_i = self.get_nearest_point_and_angle(path, Pr[0], Pr[1])
            v,w = self.calculate_control(Pr[0],Pr[1], robot_a,xi,yi,theta_i,Kd,Ka,v_max,w_max)
            self.publish_and_save_data(Pr[0], Pr[1], robot_a, v,w)
            Pr, robot_a = self.get_robot_pose()
        #
        # END OF WHILE
        #
        return

    def publish_and_save_data(self, robot_x, robot_y, robot_a, v,w):
        self.nav_data.append([robot_x, robot_y, robot_a, v, w])
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub_cmd_vel.publish(msg)
        for i in range(10):
            Pr, theta_r = self.get_robot_pose()
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
        super().__init__("stanley_node")
        self.get_logger().info("INITIALIZING PATH FOLLOWER BY STANLEY NODE ...")
        self.nav_data = []
        self.data_file = get_package_share_directory('path_follower') + "/data.txt" 
        self.robot_pose = numpy.asarray([0.0,0.0])
        self.robot_a = 0.0
        self.new_goal_pose = False
        self.goal_pose = numpy.asarray([0.0,0.0])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter('v_max', 0.5)
        self.declare_parameter('w_max', 1.0)
        self.declare_parameter('Kd', 1.0)
        self.declare_parameter('Ka', 1.0)
        self.declare_parameter('tol', 0.1)
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
                Kd    = self.get_parameter('Kd').get_parameter_value().double_value
                Ka    = self.get_parameter('Ka').get_parameter_value().double_value
                tol   = self.get_parameter('tol').get_parameter_value().double_value
                self.get_logger().info("Following path with [v_max, w_max, Kd, Ka, tol]="+str([v_max, w_max, Kd, Ka, tol]))
                path_points = [numpy.asarray([p.pose.position.x, p.pose.position.y]) for p in path.poses]
                self.stanley_path_following(path_points, Kd, Ka, v_max, w_max, tol)
                self.pub_cmd_vel.publish(Twist())
                self.pub_goal_reached.publish(Bool(data=True))
                self.get_logger().info("Global goal point reached")
                state = SM_SAVE_DATA

            elif state == SM_SAVE_DATA:
                s = ""
                for d in self.nav_data:
                    s += str(d[0]) +","+ str(d[1]) +","+ str(d[2]) +","+ str(d[3]) +","+ str(d[4]) + "\n"
                f = open(self.data_file, "w")
                f.write(s)
                f.close()
                state = SM_INIT
                
            rclpy.spin_once(self)
            self.get_clock().sleep_for(Duration(seconds=0.005))


def main(args=None):
    rclpy.init(args=args)
    stanley_node = StanleyNode()
    stanley_node.spin()
    stanley_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

