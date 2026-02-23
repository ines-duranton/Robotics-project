#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String
from ece346.lab0_truck.scripts.controller.pure_pursuit import PurePursuitController
from rclpy.executors import ExternalShutdownException

def main(args = None):
    rclpy.init(args=args)

    lab1_node = PurePursuitController()
    lab1_node.get_logger().info("Started simulator Node")

    try:    
        rclpy.spin(lab1_node)
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
        lab1_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()