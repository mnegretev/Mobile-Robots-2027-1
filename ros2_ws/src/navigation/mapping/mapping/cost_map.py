#
# MOBILE ROBOTS - FI-UNAM, 2026-2
# MAP INFLATION AND COST MAPS
#
# Instructions:
# Write the code necesary to get a cost map given an occupancy grid map and a cost radius.
# Complete the code necessary to inflate the obstacles given an occupancy grid map and
# a number of cells to inflate.
#

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import numpy

FULL_NAME = "FULL NAME"

class CostMapNode(Node):
    def get_inflated_map(self, static_map, inflation_cells):
        self.get_logger().debug("Inflating map by " + str(inflation_cells) + " cells")
        inflated = numpy.copy(static_map)
        [height, width] = static_map.shape
        #
        # TODO:
        # Write the code necessary to inflate the obstacles in the map a radius
        # given by 'inflation_cells' (expressed in number of cells)
        # Map is given in 'static_map' as a bidimensional numpy array.
        # Consider as occupied cells all cells with an occupation value greater than 50
        #
        
        return inflated
    
    def get_cost_map(self, static_map, cost_radius):
        self.get_logger().debug("Getting cost map with " + str(cost_radius) + " cells")
        cost_map = numpy.copy(static_map)
        [height, width] = static_map.shape
        #
        # TODO:
        # Write the code necessary to calculate a cost map for the given map.
        # To calculate cost, consider as example the following map:    
        # [[ 0 0 0 0 0 0]
        #  [ 0 X 0 0 0 0]
        #  [ 0 X X 0 0 0]
        #  [ 0 X X 0 0 0]
        #  [ 0 X 0 0 0 0]
        #  [ 0 0 0 X 0 0]]
        # Where occupied cells 'X' have a value of 100 and free cells have a value of 0.
        # Cost is an integer indicating how near cells and obstacles are:
        # [[ 3 3 3 2 2 1]
        #  [ 3 X 3 3 2 1]
        #  [ 3 X X 3 2 1]
        #  [ 3 X X 3 2 2]
        #  [ 3 X 3 3 3 2]
        #  [ 3 3 3 X 3 2]]
        # Cost_radius indicate the number of cells around obstacles with costs greater than zero.
        
        return cost_map

    def callback_inflated_map(self, request, response):
        response.map = self.inflated_map
        return response
        
    def callback_cost_map(self, request, response):
        response.map = self.cost_map
        return response

    def callback_timer(self):
        self.map_info   = self.map_static.info
        self.map_width  = self.map_info.width
        self.map_height = self.map_info.height
        self.map_res    = self.map_info.resolution
        self.map_data = numpy.reshape(numpy.asarray(self.map_static.data, dtype='int'), (self.map_height, self.map_width))
        inflation_radius  = self.get_parameter('inflation_radius').get_parameter_value().double_value
        inflated_map_data = self.get_inflated_map(self.map_data, round(inflation_radius/self.map_res))
        inflated_map_data = numpy.ravel(numpy.reshape(inflated_map_data, (self.map_width*self.map_height, 1)))
        cost_radius  = self.get_parameter('cost_radius').get_parameter_value().double_value
        cost_map_data = self.get_cost_map(self.map_data, round(cost_radius/self.map_res))
        cost_map_data = numpy.ravel(numpy.reshape(cost_map_data, (self.map_width*self.map_height, 1)))
        self.inflated_map = OccupancyGrid(info=self.map_info, data=inflated_map_data)
        self.inflated_map.header.frame_id = "map"
        self.inflated_map.header.stamp = self.get_clock().now().to_msg()
        self.pub_inflated_map.publish(self.inflated_map)
        self.cost_map = OccupancyGrid(info=self.map_info, data=cost_map_data)
        self.cost_map.header.frame_id = "map"
        self.cost_map.header.stamp = self.get_clock().now().to_msg()
        self.pub_cost_map.publish(self.cost_map)
        return

    def __init__(self):
        super().__init__("cost_map_node")
        self.get_logger().info("INITIALIZING MAP INFLATER AND COST MAP NODE - " + FULL_NAME)
        self.clt_static_map = self.create_client(GetMap, '/map_server/map')
        self.get_logger().info("Waiting for static map service...")
        while not self.clt_static_map.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for static map service...')
        self.get_logger().info("Static map service is now available...")
        self.get_logger().info("Trying to get first static map...")
        future = self.clt_static_map.call_async(GetMap.Request())
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.map_static = response.map
        self.get_logger().info("Got static map.")
        self.declare_parameter('inflation_radius', 0.05)
        self.declare_parameter('cost_radius', 0.05)
        self.timer = self.create_timer(1.0, self.callback_timer)
        self.get_clock().sleep_for(Duration(seconds=2.0))
        self.srv_inflate_map  = self.create_service(GetMap, '/get_inflated_map', self.callback_inflated_map)
        self.srv_cost_map  = self.create_service(GetMap, '/get_cost_map', self.callback_cost_map)
        self.pub_inflated_map = self.create_publisher(OccupancyGrid, '/inflated_map', 10)
        self.pub_cost_map = self.create_publisher(OccupancyGrid, '/cost_map', 10)


def main(args=None):
    rclpy.init(args=args)
    cost_map_node = CostMapNode()
    rclpy.spin(cost_map_node)
    cost_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
