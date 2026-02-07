#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String
from ece346.Lab0.scripts.controller.pure_pursuit import PurePursuitController
from rclpy.executors import ExternalShutdownException

def main(args = None):
    rclpy.init(args=args)

    lab0_node = PurePursuitController()
    lab0_node.get_logger().info("Started simulator Node")

    try:    
        rclpy.spin(lab0_node)
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
        lab0_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()