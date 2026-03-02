#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from rcl_interfaces.msg import SetParametersResult


import numpy as np

from ece346.Lab2.scripts.frs import FRS
from racecar_msgs.msg import OdometryArray, SetArray
from geometry_msgs.msg import Polygon, Point32
from racecar_ece346.srv import GetFRS
from rclpy.executors import ExternalShutdownException



def frs2setarray(frs):
    """
    Convert a list of FRS polygons into SetArray.
    frs: list of [P,2] numpy arrays (polygon vertices)
    """
    reachable_sets = SetArray()
    for vertices in frs:
        polygon = Polygon()
        for x, y in vertices:
            polygon.points.append(Point32(x=float(x), y=float(y)))
        reachable_sets.set_list.append(polygon)
    return reachable_sets


class DynObstacle(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_node')

        # Read parameter for topic name
        self.declare_parameter('dyn_obs_topic', '/Obstacles/Dynamic')
        self.dyn_obs_topic = self.get_parameter('dyn_obs_topic').value

        # FRS model
        self.lane_change_cost = 1
        self.frs = FRS(self)

        # Class variable to store dynamic obstacles
        self.dyn_obstacles = []

        # --- ROS 2 SUBSCRIBER ---
        self.create_subscription(OdometryArray, self.dyn_obs_topic, self.odom_cb, 1)

        # --- ROS 2 PARAMETERS ---
        self.declare_parameter('K_vx', 0.0)
        self.declare_parameter('K_vy', 0.0)
        self.declare_parameter('K_y', 0.0)
        self.declare_parameter('dx', 0.0)
        self.declare_parameter('dy', 0.0)
        self.declare_parameter('allow_lane_change', False)

        self.K_vx = 0
        self.K_vy = 0
        self.K_y = 0
        self.dx = 0
        self.dy = 0
        self.allow_lane_change = False

        # Parameter change callback
        self.add_on_set_parameters_callback(self.param_callback)

        # --- ROS 2 SERVICE ---
        self.create_service(GetFRS, '/obstacles/get_frs', self.srv_cb)

        self.get_logger().info("Dynamic Obstacle Node Initialized")

    # -------------------------
    # ROS 2 CALLBACKS
    # -------------------------

    def odom_cb(self, odom_array: OdometryArray):
        """Store incoming dynamic obstacle poses"""
        self.dyn_obstacles = odom_array.odometry_array

    def param_callback(self, params):
        """ROS 2 parameter change handler"""
        for p in params:
            if p.name == "K_vx":
                self.K_vx = p.value
            elif p.name == "K_vy":
                self.K_vy = p.value
            elif p.name == "K_y":
                self.K_y = p.value
            elif p.name == "dx":
                self.dx = p.value
            elif p.name == "dy":
                self.dy = p.value
            elif p.name == "allow_lane_change":
                self.allow_lane_change = p.value

        return SetParametersResult(successful = True)

    def srv_cb(self, req, response=None):
        """
        ROS 2 service callback: compute and return FRS for each obstacle.
        """
        t_list = req.t_list
        response = GetFRS.Response()

        for obstacle in self.dyn_obstacles:
            frs_list = self.frs.get(
                obstacle,
                t_list,
                self.K_vx,
                self.K_vy,
                self.K_y,
                self.dx,
                self.dy,
                allow_lane_change=self.allow_lane_change
            )

            for frs in frs_list:
                response.frs.append(frs2setarray(frs))

        return response


# -------------------------
# MAIN
# -------------------------
def main(args = None):
    rclpy.init(args=args)

    dyn_obs_node = DynObstacle()
    try:
        rclpy.spin(dyn_obs_node)
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
        dyn_obs_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
