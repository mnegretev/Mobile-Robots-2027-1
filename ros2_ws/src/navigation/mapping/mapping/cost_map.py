#
# MOBILE ROBOTS - FI-UNAM, 2027-1
# MAP INFLATION AND COST MAPS
#
# Instructions:
# Write the code necesary to get a cost map given an occupancy grid map and a cost radius.
# Complete the code necessary to inflate the obstacles given an occupancy grid map and
# a number of cells to inflate.
# Modify only the sections marked with the TODO comment
#

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import numpy

FULL_NAME = "Perez Salazar Alfredo"

class CostMapNode(Node):
    def get_inflated_map(self, static_map, inflation_cells):
        inflation_cells = max(0,min(inflation_cells, 10))
        self.get_logger().info("Inflating map by " + str(inflation_cells) + " cells")
        inflated = numpy.copy(static_map)
        [height, width] = static_map.shape
        #
        # TODO:
        # Write the code necessary to inflate the obstacles in the map a radius
        # given by 'inflation_cells' (expressed in number of cells)
        # Map is given in 'static_map' as a bidimensional numpy array.
        # Consider as occupied cells all cells with an occupation value greater than 50
        #
        for i in range(len(static_map)):
            for j in range(len(static_map[0])):
                if(static_map[i,j]==100):
                    for k1 in range(-inflation_cells, inflation_cells):
                        for k2 in range(-inflation_cells, inflation_cells):
                            r = min(height-1, max(0,i+k1))
                            c = min(width-1, max(0,j+k2))
                            inflated[r,c]=100

        return inflated
    
    def get_cost_map(self, static_map, cost_radius):
        cost_radius = max(0,min(cost_radius, 10))
        self.get_logger().info("Getting cost map with " + str(cost_radius) + " cells")
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
        # Cost_radius indicates the number of cells around obstacles with costs greater than zero.
        # Just for visualization purposes, the cost is multiplied by 4

        for i in range(height):
            for j in range(width):
                if static_map[i,j] ==100:
                    if static_map[i+1,j]==100 and static_map[i-1,j] == 100 and static_map[i,j+1]==100 and static_map[i,j-1] == 100:
                        continue
                    for k1 in range(-cost_radius, cost_radius+1):
                        for k2 in range(-cost_radius, cost_radius+1):
                            if (i+k1) <0 or (i+k1) >= height or (j+k2) <0 or (j+k2)>= width or static_map[i+k1,j+k2]==100:
                                continue
                            cost = 4*(cost_radius - max(abs(k1),abs(k2))+1)
                            cost_map[i+k1,j+k2]= max(cost, cost_map[i+k1,j+k2])
        
        return cost_map

    def callback_inflated_map(self, request, response):
        response.map = self.inflated_map
        return response
    
    def callback_cost_map(self, request, response):
        response.map = self.cost_map
        return response

    def get_augmented_maps(self):
        map_info   = self.map_static.info
        map_width  = map_info.width
        map_height = map_info.height
        map_res    = map_info.resolution
        
        map_data = numpy.reshape(numpy.asarray(self.map_static.data, dtype='int'), (map_height, map_width))
        inflation_radius = self.get_parameter('inflation_radius').get_parameter_value().double_value
        cost_radius  = self.get_parameter('cost_radius').get_parameter_value().double_value
        inflated_map_data = self.get_inflated_map(map_data, round(inflation_radius/map_res))
        cost_map_data = self.get_cost_map(inflated_map_data, round(cost_radius/map_res))
        
        inflated_map_data = numpy.ravel(numpy.reshape(inflated_map_data, (map_width*map_height, 1)))
        cost_map_data = numpy.ravel(numpy.reshape(cost_map_data, (map_width*map_height, 1)))
        
        self.inflated_map = OccupancyGrid(info=map_info, data=inflated_map_data)
        self.inflated_map.header.frame_id = "map"
        self.inflated_map.header.stamp = self.get_clock().now().to_msg()
        self.pub_inflated_map.publish(self.inflated_map)
        
        self.cost_map = OccupancyGrid(info=map_info, data=cost_map_data)
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
        self.pub_cost_map = self.create_publisher(OccupancyGrid, '/cost_map', 10)
        self.pub_inflated_map = self.create_publisher(OccupancyGrid, '/inflated_map', 10)
        self.get_augmented_maps()
        self.srv_inflated_map  = self.create_service(GetMap, '/get_inflated_map', self.callback_inflated_map)
        self.srv_cost_map  = self.create_service(GetMap, '/get_cost_map', self.callback_cost_map)


def main(args=None):
    rclpy.init(args=args)
    cost_map_node = CostMapNode()
    rclpy.spin(cost_map_node)
    cost_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
