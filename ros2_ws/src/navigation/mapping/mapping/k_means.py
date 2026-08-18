#
# MOBILE ROBOTS - FI-UNAM, 2027-1
# CLUSTERING BY K-MEANS
#
# Instructions:
# 
#

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy

FULL_NAME = "FULL NAME"

class KMeansNode(Node):
    def generate_random_centroids(self, K, min_x, min_y, max_x, max_y, static_map):
        centroids = numpy.zeros((K,2))
        for i in range(K):
            in_free_space = False
            while not in_free_space:
                x = (max_x - min_x) * numpy.random.rand() + min_x
                y = (max_y - min_y) * numpy.random.rand() + min_y
                c = int((x - static_map.info.origin.position.x)/static_map.info.resolution)
                r = int((y - static_map.info.origin.position.y)/static_map.info.resolution)
                idx = r*static_map.info.width + c
                in_free_space = static_map.data[idx] < 40 and static_map.data[idx] >= 0
            centroids[i,0] = x
            centroids[i,1] = y
        return centroids

    def recalculate_centroids(self, centroids, P):
        clusters = numpy.zeros(centroids.shape)
        counters = numpy.zeros((len(centroids), 1))
        new_centroids = numpy.zeros(centroids.shape)
        #
        # TODO:
        # Recalculate centroids
        #
        return new_centroids
            

    def k_means(self, P, K, static_map, tol):
        self.get_logger().info("Clustering " + str(len(P)) + "points with " + str(K) + " centroids")
        
        centroids = self.generate_random_centroids(K, -5, -5, 5, 5, static_map)
        self.pub_centroids.publish(self.get_centroids_marker(centroids))

        #
        # TODO:
        # Write the code to cluster P by K-means
        # P are the set of points to be clustered, K is the number of centroids,
        # static_map is the occupancy grid used to generate the first random centroids in the free space
        # tol is the tolerance to consider that the centroids have converged
        #
        max_distance = float("inf")
        while max_distance > tol:
            self.get_logger().info("Recalculating centroids")
            new_centroids = self.recalculate_centroids(centroids, P)
            max_distance = numpy.max([numpy.linalg.norm(centroids[i] - new_centroids[i]) for i in range(len(centroids))])
            centroids = new_centroids
            self.pub_centroids.publish(self.get_centroids_marker(centroids))
        self.get_logger().info("Converged K centroids after")
    
    def __init__(self):
        super().__init__("k_means_node")
        self.get_logger().info("INITIALIZING K-MEANS NODE - " + FULL_NAME)
        self.clt_inflated_map = self.create_client(GetMap, '/get_inflated_map')
        self.get_logger().info("Waiting for inflated map service...")
        while not self.clt_inflated_map.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for inflated map service...')
        self.get_logger().info("Inflated map service is now available...")
        future = self.clt_inflated_map.call_async(GetMap.Request())
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.static_map = response.map
        self.get_logger().info("Got inflated map.")
        self.declare_parameter('K', 3)
        self.declare_parameter('tol', 0.2)
        self.pub_centroids = self.create_publisher(Marker, '/mapping/k_means_centroids', 10)

    def get_centroids_marker(self, centroids):
        mrk = Marker()
        mrk.header.frame_id = "map"
        mrk.header.stamp = self.get_clock().now().to_msg()
        mrk.ns = "mapping"
        mrk.lifetime.sec=10000
        mrk.id = 0
        mrk.type   = Marker.SPHERE_LIST
        mrk.action = Marker.ADD
        mrk.scale.x, mrk.scale.y, mrk.scale.z = 0.2, 0.2, 0.2
        mrk.color.r, mrk.color.a = 1.0, 1.0
        mrk.points = [Point(x=c[0], y=c[1], z=0.1) for c in centroids]
        return mrk

    def get_cartesian_free_points(self, static_map):
        P = []
        for r in range(static_map.info.height):
            for c in range(static_map.info.width):
                idx = r*static_map.info.width + c
                if static_map.data[idx] >= 0 and static_map.data[idx] < 40:
                    x = c*static_map.info.resolution + static_map.info.origin.position.x
                    y = r*static_map.info.resolution + static_map.info.origin.position.y
                    P.append([x,y])
        return numpy.asarray(P)
        

    def spin(self):
        K = self.get_parameter('K').get_parameter_value().integer_value
        tol = self.get_parameter('tol').get_parameter_value().double_value
        P = self.get_cartesian_free_points(self.static_map)
        self.k_means(P, K, self.static_map, tol)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            self.get_clock().sleep_for(Duration(seconds=0.1))

def main(args=None):
    rclpy.init(args=args)
    k_means_node = KMeansNode()
    k_means_node.spin()
    k_means_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
