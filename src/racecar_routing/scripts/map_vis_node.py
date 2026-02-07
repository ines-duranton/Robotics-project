#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

import pickle
from routing.routing.utils import map_to_markerarray

from ament_index_python.packages import get_package_share_directory
from pathlib import Path as FsPath

import lanelet2
from lanelet2.projection import LocalCartesianProjector

from rclpy.executors import ExternalShutdownException


"""
Node that will output the Map MarkerArray
"""
def main(args = None):
    rclpy.init(args = args)

    map_node = Node("map_node")

    try:
        map_node.get_logger().info("Initialized map_publisher_node")

        #projector for lanelet_map from 

        #declare Map
        try:
            share_dir = FsPath(get_package_share_directory('racecar_routing'))
        except Exception as e:
            map_node.get_logger().error(f"Could not locate package 'racecar_routing' share dir: {e}")
            map_node.destroy_node()
            rclpy.shutdown()
            return

        
        default_map = share_dir / 'maps' / 'track.osm'

        map_node.declare_parameter('map_file', str(default_map))
        
        map_file = map_node.get_parameter('map_file').value

        projector  = LocalCartesianProjector(lanelet2.io.Origin(0, 0, 0))

        try:
            lanelet_map = lanelet2.io.load(map_file, projector)
        except Exception as e:
            map_node.get_logger().error(f"Failed to load map_file '{map_file}': {e}")
            map_node.destroy_node()
            rclpy.shutdown()
            return

        map_pub = map_node.create_publisher(MarkerArray, "/Routing/Map", 10)

        map_message = map_to_markerarray(lanelet_map)

        def timer_callback():
            map_pub.publish(map_message)

        #publish the MarkerArray of the Map every 1 seconds
        map_node.create_timer(1, timer_callback)

        rclpy.spin(map_node)
    
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException: #context shutdown from outside, ros2 launch
        pass
    except Exception as e:
        #if shutdown race
        if not rclpy.ok():
            pass
        else:
            raise
    finally:
        map_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    

if __name__ == '__main__':
     main()