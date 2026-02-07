#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interface.simulator.simulator import Simulator
from rclpy.executors import ExternalShutdownException
def main(args=None):
    rclpy.init(args=args)

    simulator_node = Simulator()
    simulator_node.get_logger().info("Started simulator Node")

    try:    
        rclpy.spin(simulator_node)
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
        simulator_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        
    

if __name__ == '__main__':
    main()