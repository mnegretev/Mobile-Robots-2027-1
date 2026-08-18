#
# MOBILE ROBOTS - FI-UNAM, 2026-2
# RAPIDLY EXPLORING RANDOM TREES
#
# Instructions:
# Write the code necessary to plan a path using an
# occupancy grid and the RRT algorithm
# MODIFY ONLY THE SECTIONS MARKED WITH THE 'TODO' COMMENT
#
import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from geometry_msgs.msg import PoseStamped, Pose, Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from nav_msgs.srv import *
from builtin_interfaces.msg import Duration
from collections import deque
import numpy
import heapq
import math

NAME = "FULL NAME"

class TreeNode:
    def __init__(self, x, y, parent=None):
        self.children = []
        self.parent = parent
        self.x = x
        self.y = y

class RRTNode(Node):
    def in_free_space(self, x,y,grid_map):
        c = int((x - grid_map.info.origin.position.x)/grid_map.info.resolution)
        r = int((y - grid_map.info.origin.position.y)/grid_map.info.resolution)
        return grid_map.data[r*grid_map.info.width + c] < 40 and grid_map.data[r*grid_map.info.width + c] >= 0
    
    def get_random_q(self, grid_map):
        min_x = grid_map.info.origin.position.x
        min_y = grid_map.info.origin.position.y
        max_x = min_x + grid_map.info.width *grid_map.info.resolution
        max_y = min_y + grid_map.info.height*grid_map.info.resolution
        is_free = False
        attempts = len(grid_map.data)
        while not is_free and attempts > 0:
            x = numpy.random.uniform(min_x, max_x)
            y = numpy.random.uniform(min_y, max_y)
            is_free = self.in_free_space(x,y,grid_map)
            attempts -= 1
        return [x,y]

    def get_nearest_node(self,tree, x, y):
        S = [tree] #Stack to traverse tree
        N = []     #List of all nodes
        while len(S) > 0:
            n = S.pop()
            N.append(n)
            for c in n.children:
                S.append(c)
        distances = numpy.asarray([math.sqrt((x - n.x)**2 + (y - n.y)**2) for n in N])
        return N[numpy.argmin(distances)]
    
    def get_new_node(self, nearest_node, rnd_x, rnd_y, epsilon):
        dist = math.sqrt((nearest_node.x - rnd_x)**2 + (nearest_node.y - rnd_y)**2)
        mag = min(dist, epsilon)
        if (dist == 0):
            return None
        nearest_x = nearest_node.x + mag*(rnd_x - nearest_node.x)/dist
        nearest_y = nearest_node.y + mag*(rnd_y - nearest_node.y)/dist
        return TreeNode(nearest_x, nearest_y, nearest_node)
    
    def check_collision(self, n1, n2, grid_map, epsilon):
        dist = math.sqrt((n1.x-n2.x)**2 + (n1.y-n2.y)**2)
        if dist > epsilon:
            return True
        n = 2*int(max(abs(n2.x-n1.x), abs(n2.y-n1.y))/grid_map.info.resolution)
        P = numpy.linspace([n1.x,n1.y], [n2.x,n2.y], n)
        for x,y in P:
            if not self.in_free_space(x,y,grid_map):
                return True
        return False

    def rrt(self, start_x, start_y, goal_x, goal_y, grid_map, epsilon, max_attempts):
        tree = TreeNode(start_x, start_y, None)
        goal_node = TreeNode(goal_x, goal_y, None)
    
        #
        # TODO
        # Implement the RRT algorithm for path planning
        # The tree is already created with the corresponding starting node.
        # Goal node is also already created.
        # Return both, the tree and the path. You can follow these steps:
        #
        
        path = []
        while goal_node is not None:
            path.insert(0, [goal_node.x, goal_node.y])
            goal_node = goal_node.parent
        return tree, path

    def get_tree_marker(self,tree):
        mrk = Marker()
        mrk.header.stamp = self.get_clock().now().to_msg()
        mrk.header.frame_id = "map"
        mrk.ns = "path_planning"
        mrk.lifetime = Duration(sec=10, nanosec=0)
        mrk.id = 0
        mrk.type   = Marker.LINE_LIST
        mrk.action = Marker.ADD
        mrk.color.r = 178.0/255.0#0.3
        mrk.color.g = 102.0/255.0#0.5
        mrk.color.b = 1.0
        mrk.color.a = 0.9
        mrk.scale.x = 0.03
        mrk.pose.orientation.w = 1.0
        S = [tree] #Stack to traverse tree
        while len(S) > 0:
            n = S.pop()
            for c in n.children:
                mrk.points.append(Point(x=n.x, y=n.y, z=0.0))
                mrk.points.append(Point(x=c.x, y=c.y, z=0.0))
                S.append(c)
        return mrk

    def get_inflated_map(self):
        self.get_logger().info("Trying to get inflated map...")
        future = self.clt_inflated_map.call_async(GetMap.Request())
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info("Got inflated map.")
        return response.map
   
    def callback_rrt(self, req, resp):
        [sx, sy] = [req.start.pose.position.x, req.start.pose.position.y]
        [gx, gy] = [req.goal .pose.position.x, req.goal .pose.position.y]
        epsilon  = self.get_parameter('epsilon').get_parameter_value().double_value
        max_attempts = self.get_parameter('max_n').get_parameter_value().integer_value
        str_data = str([sx,sy]) + " to " + str([gx,gy]) +" with e=" + str(epsilon) +  " and " + str(max_attempts) + " attempts."
        self.get_logger().info("Planning by RRT from " + str_data)
        
        start_time = self.get_clock().now()
        tree, path = self.rrt(sx, sy, gx, gy, self.grid_map, epsilon, max_attempts)
        end_time   = self.get_clock().now()
        
        delta_ms = (end_time.nanoseconds - start_time.nanoseconds)/1e6
        if len(path) > 0:
            self.get_logger().info("Path planned after " + str(delta_ms) + " ms")
        else:
            self.get_logger().info("Cannot plan path from  " + str([sx, sy])+" to "+str([gx, gy]) + " :'(")
        
        
        self.msg_tree = self.get_tree_marker(tree)
        self.msg_path = Path()
        self.msg_path.header.frame_id = "map"
        self.msg_path.header.stamp = self.get_clock().now().to_msg()
        self.msg_path.poses = []
        for [x,y] in path:
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            self.msg_path.poses.append(pose_stamped)
        resp.plan = self.msg_path
        return resp

    def callback_timer(self):
        self.pub_path.publish(self.msg_path)
        self.pub_tree.publish(self.msg_tree)

    def __init__(self):
        super().__init__("rrt_node")
        self.get_logger().info("INITIALIZING RRT NODE - " + NAME)
        self.clt_inflated_map = self.create_client(GetMap, '/get_inflated_map')
        self.get_logger().info("Waiting for inflated map service...")
        while not self.clt_inflated_map.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for inflated map service...')
        self.get_logger().info("Inflated map service is now available...")
        self.grid_map = self.get_inflated_map()
        self.declare_parameter('epsilon', 1.0)
        self.declare_parameter('max_n', 100)
        self.srv_plan_path = self.create_service(GetPlan, '/path_planning/plan_path', self.callback_rrt)
        self.pub_path = self.create_publisher(Path, '/path_planning/path', 10)
        self.pub_tree = self.create_publisher(Marker, '/path_planning/rrt_tree', 10)
        self.msg_path = Path()
        self.msg_tree = self.get_tree_marker(TreeNode(0,0))
        self.timer = self.create_timer(0.5, self.callback_timer)
            
def main(args=None):
    rclpy.init(args=args)
    rrt_node = RRTNode()
    rclpy.spin(rrt_node)
    rrt_node.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
