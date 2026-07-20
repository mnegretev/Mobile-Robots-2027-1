#
# MOBILE ROBOTS - FI-UNAM, 2026-2
# PATH SMOOTHING BY GRADIENT DESCEND
#
# Instructions:
# Write the code necessary to smooth a path using the gradient descend algorithm.
# MODIFY ONLY THE SECTIONS MARKED WITH THE 'TODO' COMMENT
#
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Point
from navig_msgs.srv import ProcessPath
import numpy

NAME = "FULL NAME"

class PathSmoothingNode(Node):
    def smooth_path(self, Q, w1, w2, max_steps):
        #
        # TODO:
        # Write the code to smooth the path Q, using the gradient descend algorithm,
        # and return a new smoothed path P.
        # Path is composed of a set of points [x,y] as follows:
        # [[x0,y0], [x1,y1], ..., [xn,ym]].
        # The smoothed path must have the same shape.
        # Return the smoothed path.
        #
        P = numpy.copy(Q)
        tol     = 0.00001                   
        nabla   = numpy.full(Q.shape, float("inf"))
        epsilon = 0.1                       
        
        return P

    def callback_smooth_path(self, request, response):
        w1  = self.get_parameter('w1').get_parameter_value().double_value
        w2  = self.get_parameter('w2').get_parameter_value().double_value
        steps  = self.get_parameter('steps').get_parameter_value().integer_value
        self.get_logger().info("Smoothing path with params: " + str([w1, w2, steps]))
        start_time = self.get_clock().now()
        Q = numpy.asarray([[p.pose.position.x, p.pose.position.y] for p in request.path.poses])
        P = self.smooth_path(Q, w1, w2, steps)
        end_time = self.get_clock().now()
        delta_ms = (end_time.nanoseconds - start_time.nanoseconds)/1e6
        self.get_logger().info("Path smoothed after " + str(delta_ms) + " ms")
        self.msg_smooth_path.header.frame_id = request.path.header.frame_id
        self.msg_smooth_path.header.stamp = self.get_clock().now().to_msg()
        self.msg_smooth_path.poses = []
        for i in range(len(request.path.poses)):
            p = PoseStamped()
            p.pose.position.x = P[i,0]
            p.pose.position.y = P[i,1]
            self.msg_smooth_path.poses.append(p)
        self.pub_smooth_path.publish(self.msg_smooth_path)
        response.processed_path = self.msg_smooth_path
        return response
            
    def __init__(self):
        super().__init__("path_smoothing_node")
        self.get_logger().info("INITIALIZING PATH SMOOTHING NODE - " + NAME)
        self.declare_parameter('w1', 0.9)
        self.declare_parameter('w2', 0.1)
        self.declare_parameter('steps', 10000)
        self.srv_smooth_path = self.create_service(ProcessPath, '/path_planning/smooth_path', self.callback_smooth_path)
        self.pub_smooth_path = self.create_publisher(Path, '/path_planning/smoothed_path', 10)
        self.msg_smooth_path = Path()
            
def main(args=None):
    rclpy.init(args=args)
    path_smoothing_node = PathSmoothingNode()
    rclpy.spin(path_smoothing_node)
    path_smoothing_node.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
