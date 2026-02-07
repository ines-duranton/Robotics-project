#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interface.simulator.traffic_simulator import TrafficSimulator
from rclpy.executors import ExternalShutdownException

def main(args=None):
    rclpy.init(args=args)

    traffic_node = TrafficSimulator()
    traffic_node.get_logger().info("Started traffic simulation node")

    try:    
        rclpy.spin(traffic_node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException: #context shutdown from outside, ros2 launch
        pass
    except Exception as e:
        #ignore shutdown races
        if not rclpy.ok():
            pass
        else:
            raise
    finally:
        traffic_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        
    

if __name__ == '__main__':
    main()