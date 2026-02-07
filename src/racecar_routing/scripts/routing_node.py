#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from routing.routing.routing import Routing
from rclpy.executors import ExternalShutdownException

def main(args=None):
    rclpy.init(args=args)

    routing_node = Routing()

    routing_node.get_logger().info("Started Routing Node")

    try:
        rclpy.spin(routing_node)
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
        routing_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()