#!/usr/bin/env python3
"""
Final Project — Safety Filter (STUDENT SKELETON)

You will build a ROS2 node that sits between a human driver (PS4 joystick)
and the vehicle, intervening only when necessary to keep the car safe.

Pipeline:

    /joy --> joy_to_ackermann --> /teleop ┐
                                           │
                             /SLAM/Pose ───┤---> [safety_filter_node] ---> /drive
                                           │
                       /Obstacles/Static ──┘

Topics you will work with:

    /teleop            ackermann_msgs/AckermannDriveStamped  (human command in)
    <odom_topic>       nav_msgs/Odometry                     (vehicle state in)
    /Obstacles/Static  visualization_msgs/MarkerArray        (obstacles in)
    /drive             ackermann_msgs/AckermannDriveStamped  (safe command out)

------------------------------------------------------------------------------
Task 1: Build the node plumbing (subscribers, publisher, timer).
Task 2: Implement the safety filter logic.
------------------------------------------------------------------------------

You may:
  - Add imports, helper methods, and ROS parameters to THIS file.
  - Add your own files anywhere under FinalProject/ — e.g. an `ilqr/`
    subfolder with your ILQR solver, an `ilqr_params.yaml` with cost
    weights, utility modules, etc. Load yaml configs from within this
    node using `open()` + `yaml.safe_load()`, or declare a ROS param
    for the config path and set it in `final_project_*.yaml`.
  - Edit any yaml under `FinalProject/config/` — add parameters, tune
    thresholds, change topic names. All existing yaml values are
    documented and intended to be tunable.

You should NOT need to rewrite the launch files, the plumbing nodes
(joy_to_ackermann, drive_to_servo, etc.), or the CMakeLists top-level
install lists. Only touch those if you are adding a new standalone
executable — in which case ask first.
"""

import math
import queue
import numpy as np
import os

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path as PathMsg # I think this is the /Routing/Path
from geometry_msgs.msg import PoseStamped


from scipy.spatial.transform import Rotation as R

from ament_index_python.packages import get_package_share_directory

from pathlib import Path as FsPath

from ece346.FinalProject.ILQR_Example.ref_path import RefPath
from ece346.FinalProject.ILQR_Example.ilqr import ILQR
from ece346.Lab3.scripts.utils.static_obstacle import get_obstacle_vertices

from ece346.FinalProject.ILQR_Example.cost.state_cost import StateCost
from ece346.FinalProject.ILQR_Example.cost.control_cost import ControlCost
from ece346.FinalProject.ILQR_Example.cost.obstacle_cost import ObstacleCost

from src.racecar_routing.routing.routing.lanelet_wrapper import LaneletWrapper

# from ece346.FinalProject.config import config


def yaw_from_quat(qx, qy, qz, qw):
    """Extract yaw (heading, rad) from a quaternion. Useful for Task 2."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

# helper function to compute the next state
def dyn_step(x, u, dt):
    dx = np.array([x[2]*np.cos(x[3]),
            x[2]*np.sin(x[3]),
            u[0],
            x[2]*np.tan(u[1]*1.1)/0.257,
            0])
    x_new = x + dx*dt
    x_new[2] = max(0, x_new[2]) # do not allow negative velocity
    x_new[3] = np.mod(x_new[3] + np.pi, 2 * np.pi) - np.pi
    x_new[-1] = u[1]
    return x_new

def path_callback(path_msg):
        x = []
        y = []
        width_L = []
        width_R = []
        speed_limit = []
        
        for waypoint in path_msg.poses:
            x.append(waypoint.pose.position.x)
            y.append(waypoint.pose.position.y)
            width_L.append(waypoint.pose.orientation.x)
            width_R.append(waypoint.pose.orientation.y)
            speed_limit.append(waypoint.pose.orientation.z)
                    
        centerline = np.array([x, y])
        
        ref_path = RefPath(centerline, width_L, width_R, speed_limit, loop=False)
        return ref_path


class SafetyFilterNode(Node):

    # =========================================================================
    # TASK 1 — Node setup (subscribers, publisher, timer)
    # =========================================================================
    #
    # Fill in __init__ below so that the node:
    #   1. Declares ROS parameters for each topic name and for the publish rate.
    #      Hint: use self.declare_parameter('name', default_value). The yaml
    #      file (final_project_*.yaml) will override these at launch time.
    #      Required parameter names (match the yaml):
    #          teleop_topic, drive_topic, odom_topic, static_obs_topic,
    #          publish_rate
    #
    #   2. Creates three subscribers that cache the latest message each:
    #          /teleop            -> AckermannDriveStamped
    #          <odom_topic>       -> Odometry
    #          /Obstacles/Static  -> MarkerArray
    #      Hint: self.create_subscription(MsgType, topic, callback, queue_size)
    #      Each callback can be a one-liner that stores msg into an instance
    #      attribute (e.g. self._latest_teleop = msg).
    #
    #   3. Creates one publisher:
    #          /drive             -> AckermannDriveStamped
    #      Hint: self.create_publisher(MsgType, topic, queue_size)
    #
    #   4. Creates a timer at `publish_rate` Hz that calls a method which
    #      invokes self.safety_filter(...) and publishes the result.
    #      Hint: self.create_timer(period_sec, callback)
    #
    # For reference, open any other node in this repo (e.g.
    # FinalProject/scripts/joy_to_ackermann_node.py) to see the same pattern.
    # =========================================================================

    def __init__(self):
        super().__init__('safety_filter_node')

        # ---- TODO(Task 1.1): declare ROS parameters ----
        self.declare_parameter('teleop_topic')
        self.declare_parameter('drive_topic')
        self.declare_parameter('odom_topic')
        self.declare_parameter('static_obs_topic')  
        self.declare_parameter('publish_rate')

        self.package = get_package_share_directory("racecar_ece346")

        self.declare_parameter('monitoring_ilqr_params_file', os.path.join(self.package, "config", "final_truck_ilqr.yaml"))
        self.declare_parameter('safety_ilqr_params_file', os.path.join(self.package, "config", "safe_ilqr.yaml"))

        try:
            share_dir = FsPath(get_package_share_directory('racecar_routing'))
        except Exception as e:
            self.get_logger().error(f"Could not locate package 'Routing' share dir: {e}")
            share_dir = FsPath('.')  # dummy to avoid NameError
        
        default_map = share_dir / 'maps' / 'track.osm'
        
        self.declare_parameter('map_file', str(default_map))
        self.declare_parameter('lane_change_cost', 0)

        # For routing path
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('routing_path_topic', '/Routing/Path')

        self.declare_parameter('projection_dt')
        self.declare_parameter('total_soft_threshold') # added to make it easy to change

        # self.virtual_goals = [(5.4, 5.3), (0.65, 5.61), (2.8, 0.6), (2.8, 5.6), (0.72, 0.53), (5.47, 0.59)]
        self.virtual_goals = [(0.28, 5.37),(0.8, 0.68),(5.45, 0.66),(5.43, 5.62)]
        self.virtual_goal_idx = 0
        self.virtual_goal_radius = 3

        # ---- TODO(Task 1.2): read parameter values ----

        teleop_topic1 = self.get_parameter('teleop_topic').value     # AckermannDriveStamped
        odom_topic = self.get_parameter('odom_topic').value      # Odometry
        obs_topic = self.get_parameter('static_obs_topic').value        # MarkerArray
        drive_topic = self.get_parameter('drive_topic').value
        goal_topic = self.get_parameter('goal_topic').value # Pose
        routing_topic = self.get_parameter('routing_path_topic').value
        self.monitoring_ilqr_params_file = self.get_parameter('monitoring_ilqr_params_file').value
        self.safety_ilqr_params_file = self.get_parameter('safety_ilqr_params_file').value

        self.publish_rate = self.get_parameter('publish_rate').value

        self.dt = self.get_parameter('projection_dt').value

        map_file = self.get_parameter('map_file').value
        self.lane_change_cost = self.get_parameter('lane_change_cost').value

        self.total_soft_threshold = self.get_parameter('total_soft_threshold').value
        self.safe_release_threshold = 0.5 * self.total_soft_threshold               # ==================== Do you wanna use something like this not to make the car jittery?
        self.safe_trigger = False # not 1 or 0 as i initially said in the notes

        self.last_steering_angle = 0.0 #keep track of the steering angle

        self.teleop_current = None
        self.odom_current = None
        self.obs_stat_current = None
        self.goal_current = None
        self.goal_routing = None

        self.obstacle_list = []

        self.ref_path = None

        self.user_route = LaneletWrapper(map_file, self)
        self.monitoring_planner = ILQR(logger = self.get_logger(), config_file = self.monitoring_ilqr_params_file)
        self.safety_planner = ILQR(logger = self.get_logger(), config_file = self.safety_ilqr_params_file)

        # ---- TODO(Task 1.3): create subscribers ----
        self.create_subscription(AckermannDriveStamped, teleop_topic1, self.teleop_cb, 1)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 1)
        self.create_subscription(MarkerArray, obs_topic, self.obs_stat_cb, 1)
        #self.create_subscription(PoseStamped, goal_topic, self.goal_cb, 1)
        self.create_subscription(PathMsg, routing_topic, self.routing_cb, 1)

        # ---- TODO(Task 1.4): create the publisher ----
        self.pub = self.create_publisher(AckermannDriveStamped, drive_topic, 1)
        self.pub_goal = self.create_publisher(PoseStamped, goal_topic, 1)

        # ---- TODO(Task 1.5): create a timer at publish_rate Hz ----
        self.create_timer(1/self.publish_rate, self._publish_filtered)

        self.get_logger().info(
            f"Publish rate: {self.publish_rate}"
        )

        self.get_logger().info(
            f"safety_filter_node ready: {teleop_topic1} + {odom_topic} "
            f"+ {obs_topic} + {goal_topic} -> {drive_topic}"
        )

    # ---- TODO(Task 1.6): implement callbacks ----
    def teleop_cb(self, msg: AckermannDriveStamped):
        self.teleop_current = msg
    
    def odom_cb(self, msg: Odometry):
        self.odom_current = msg
    
    #makes obstacles update live and avoids rebuilding the reference path every timer tick
    def obs_stat_cb(self, msg: MarkerArray):
        self.obs_stat_current = msg
        self.obstacle_list = []

        # like in traj_planner_example obstacle callback
        for obs in msg.markers:
            _, vertices = get_obstacle_vertices(obs)
            self.obstacle_list.append(vertices)

    def goal_cb(self, msg: PoseStamped):
        self.goal_current = msg


    def routing_cb(self, msg: PathMsg):
        self.goal_routing = msg
        try:
            self.ref_path = path_callback(msg)
        except Exception:
            self.ref_path = None


    # ---- TODO(Task 1.7): the timer callback ----
    # Should:
    #   - return early if no teleop has arrived yet (self._latest_teleop is None)
    #   - call self.safety_filter(teleop=..., odom=..., obstacles=...)
    #   - if the returned command is not None, update its header.stamp to now
    #     and publish it on /drive
    #
    def _publish_filtered(self):
        if self.teleop_current == None:
            return
        else:
            command = self.safety_filter(self.teleop_current, self.odom_current, self.obs_stat_current, self.goal_current, self.goal_routing)
            if command != None:
                command.header.stamp = self.get_clock().now().to_msg()
                self.last_steering_angle = command.drive.steering_angle # added to apply the bicyle model to the last angle state not the joystick request
                command.drive.speed = max(0.0, min(command.drive.speed, 0.3))
                self.pub.publish(command)
            return


    # =========================================================================
    # TASK 2 — Safety filter implementation
    # =========================================================================
    #
    # Start as a passthrough, then add real safety logic.
    #
    # You are free to pick any approach (or your own). Add helper methods,
    # extra parameters, even a sub-folder of modules — this skeleton will
    # get out of the way.
    # =========================================================================

    # values to keep track of the previous control command (TODO: make accessible in safety_filter())
    prev_state = None #[x, y, v, psi, delta]
    prev_u = np.zeros(3) # [accel, steer, t]


    def safety_filter(self, teleop, odom, obstacles, goal, routing):
        """
        Args
        ----
        teleop : ackermann_msgs.msg.AckermannDriveStamped   (or None)
            Human's desired command. Useful fields:
                teleop.drive.speed           float   m/s     target forward speed
                teleop.drive.steering_angle  float   rad     target steering angle
                teleop.drive.acceleration    float   m/s^2   usually 0 from the joy
                teleop.header.stamp          Time            when the command was issued

        odom : nav_msgs.msg.Odometry                        (or None)
            Vehicle state. Useful fields:
                odom.pose.pose.position.x       float   m       map-frame x
                odom.pose.pose.position.y       float   m       map-frame y
                odom.pose.pose.position.z       float   m       usually 0
                odom.pose.pose.orientation      Quaternion (.x .y .z .w)
                    → use yaw_from_quat(..) above to get heading in rad
                odom.twist.twist.linear.x       float   m/s     forward velocity
                odom.twist.twist.angular.z      float   rad/s   yaw rate

        obstacles : visualization_msgs.msg.MarkerArray      (or None)
            Static obstacles (cubes). Useful fields:
                obstacles.markers               list[Marker]
                for m in obstacles.markers:
                    m.pose.position.x / .y / .z float   m       obstacle center
                    m.scale.x / .y / .z         float   m       cube size (x=y=z typically)
                    m.id                        int             obstacle id
                    m.ns                        str             namespace

        Returns
        -------
        ackermann_msgs.msg.AckermannDriveStamped
            The command to publish on /drive. Set `.drive.speed` (m/s) and
            `.drive.steering_angle` (rad). Header.stamp is overwritten for you.
            Return None to skip publishing this tick.
        """
        # ---- TODO(Task 2): replace this passthrough ----

        # Information about the /Routing/Path topic: (run "ros2 interface show nav_msgs/msg/Path" in another terminal that's inside container while sim running)
        # # An array of poses that represents a Path for a robot to follow.

        # # Indicates the frame_id of the path.
        # std_msgs/Header header

        # # Array of poses to follow.
        # geometry_msgs/PoseStamped[] poses

        # I think we can use poses[] to find the goal points. This is the straight line path the robot should follow

        # # Using /Routing/Path to set goal points
        # if goal is not None:
        #     x_goal, y_goal = goal.pose.position.x, goal.pose.position.y
        if teleop is None or odom is None or obstacles is None:
            return teleop
        
        
        # if teleop is None:
            
        #     return None

        # if odom is None:
        #     return teleop
        
        # if routing is None:
        #     return teleop

        # Get current state and control actions from ROS topics
        initial_yaw = yaw_from_quat(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
        initial_x = odom.pose.pose.position.x
        initial_y = odom.pose.pose.position.y
        initial_v = max(0.0, odom.twist.twist.linear.x) # dunnot if fix, but we also do it in the dym step
        initial_steering_angle = self.last_steering_angle #teleop.drive.steering_angle

        inner_square_x = [1.40, 5.33]
        inner_square_y = [1.4, 5.3]
        turn_region_x = [2.75, 3.3]
        
        if inner_square_x[0] < initial_x and initial_x < inner_square_x[1] and inner_square_y[0] < initial_y and initial_y < inner_square_y[1] :
            self.get_logger().info(f"in square, ilqr off")
            return teleop

        if initial_y >= inner_square_y[1] and turn_region_x[0] < initial_x and initial_x < turn_region_x[1] and teleop.drive.steering_angle > 0.1 : 
            self.get_logger().info(f"turning into turning region")
            return teleop
        
        car_xy = np.array([initial_x, initial_y])

        # self.get_logger().info(f"teleop: {teleop}")

        goal_selected = self.virtual_goals[int(np.argmax(np.linalg.norm(self.virtual_goals - car_xy, axis=1)))]

        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal_selected[0]
        goal_msg.pose.position.y = goal_selected[1]
        
        #print(f"Published {goal_selected[0]}, {goal_selected[1]} goal")
        self.pub_goal.publish(goal_msg)

        user_ref_path = self.ref_path

        if routing is None:
            return teleop
        
        user_ref_path = path_callback(routing)
        # car_xy = np.array([initial_x, initial_y])
        # goals = np.array(self.virtual_goals, dtype=float)

        # if routing is None:
        #     goal_selected = goals[int(np.argmax(np.linalg.norm(goals - car_xy, axis=1)))]

        #     goal_msg = PoseStamped()
        #     goal_msg.header.stamp = self.get_clock().now().to_msg()
        #     goal_msg.header.frame_id = "map"
        #     goal_msg.pose.position.x = float(goal_selected[0])
        #     goal_msg.pose.position.y = float(goal_selected[1])
        #     goal_msg.pose.orientation.w = 1.0

        #     self.pub_goal.publish(goal_msg)
        #     return teleop

        # user_ref_path = path_callback(routing)
        # Create np arrays for the state and control
        initial_state = np.array([initial_x, initial_y, initial_v, initial_yaw, initial_steering_angle])
        #initial_control = np.array([initial_acc, initial_steering_angle]) 
        # dyn_step uses u = [accel, steering_angle].

        # start_pose = goal.pose
        # start_pose.position.x, start_pose.position.y= state_after_user_control[0], state_after_user_control[1]
        # reference_path = self.user_route.get_shortest_path(start_pose, goal.pose)
        
        # Update obstacles

        obstacle_list = []

        # like in traj_planner_example obstacle callback
        for obs in obstacles.markers:
            _, vertices = get_obstacle_vertices(obs)
            obstacle_list.append(vertices)

        # Set up monitoring ILQR planner
        self.monitoring_planner.update_obstacles(obstacle_list)
        #user_ref_path = path_callback(routing) # Centerline routing path
        self.monitoring_planner.update_ref_path(user_ref_path)

        # Roll out user trajectory and score THIS trajectory directly
        user_trajectory = np.zeros((5, self.monitoring_planner.T))
        user_controls = np.zeros((2, self.monitoring_planner.T))
    
        user_trajectory[:, 0] = initial_state
        state_after_user_control = initial_state.copy() # do not mutate the state 
    
        for i in range(self.monitoring_planner.T - 1):
            target_speed = teleop.drive.speed        
            target_steering = teleop.drive.steering_angle
            #target_acc = teleop.drive.acceleration
            target_acc = (target_speed- state_after_user_control[2])/self.dt

            #target_acc = np.clip(target_acc, -1.0, 1.0)
            #target_steering = np.clip(target_steering, -0.34, 0.34)  
    
            initial_control = np.array([target_acc, target_steering])
            user_controls[:, i] = initial_control
            state_after_user_control = dyn_step(state_after_user_control, initial_control, self.dt)
            user_trajectory[:, i + 1] = state_after_user_control

        user_controls[:, -1] = user_controls[:, -2]

        # Get cost of the human/user trajectory
        path_refs, obs_refs = self.monitoring_planner.get_references(user_trajectory)
        user_cost = self.monitoring_planner.cost.get_traj_cost(user_trajectory, user_controls, path_refs, obs_refs)
        user_cost = float(user_cost) #just in case for the comparison, even if i think it already returns a float
        #print('cost :', user_cost)
        # user_state_cost = self.monitoring_planner.cost.state_cost.get_traj_cost(user_trajectory, user_controls, path_refs)
        user_obstacle_cost = self.monitoring_planner.cost.obstacle_cost.get_traj_cost(user_trajectory, user_controls, obs_refs)

        # #Set up safety ILQR planner (this line is taken from traj_planner_example.py)

        self.safety_planner.update_obstacles(obstacle_list)
        user_ref_path = path_callback(routing) # Centerline routing path
        self.safety_planner.update_ref_path(user_ref_path)

        # # if not np.all(np.isfinite(state_after_user_control)):
        # #     self.get_logger().warn("State has NaN/Inf, skipping iLQR")
        # #     return teleop
        # # if not np.all(np.isfinite(user_trajectory)):
        # #     return teleop
        
        # user_plan = self.safety_planner.plan(state_after_user_control, None)
        # if user_plan is None:
        #     return teleop

        # plan_status = user_plan['status']
        # if plan_status == -1:
        #     print("User planning failed")
        #     self.get_logger().info(
        #         f"user planning failed"
        #     )
        #     return None #TODO: maybe return something else?

        # # Did ILQR actually return a plan from the future state?
        # if user_plan is not None:
        #     # Get cost of planned trajectory
        #     path_refs, obs_refs = self.safety_planner.get_references(user_plan['trajectory'])
        #     recovery_cost = self.safety_planner.cost.get_traj_cost(user_plan['trajectory'], user_plan['controls'], path_refs, obs_refs)
        #     recovery_state_cost = self.safety_planner.cost.state_cost.get_traj_cost(user_plan['trajectory'], user_plan['controls'], path_refs)
        #     recovery_obstacle_cost = self.safety_planner.cost.obstacle_cost.get_traj_cost(user_plan['trajectory'], user_plan['controls'], obs_refs)

        # If below (very arbitary) threshold, publish user control
        # if user_obstacle_cost > 190:
        #     brake_command = AckermannDriveStamped()
        #     brake_command.drive.speed = 0.0
        #     brake_command.drive.steering_angle = 0.0
        #     return brake_command
        
        # self.get_logger().info(
        #             f"User cost: {user_cost}"
        #         )
        if user_cost < 8: #220: #user_state_cost < 30 and user_obstacle_cost < 40:
            #print("Running teleop because user plan cost is low")
            return teleop
        else:
            # If the future plan is too expensive, use safety ILQR from the current state

            #if routing is not None:
            self.safety_planner.update_obstacles(obstacle_list)
            self.safety_planner.update_ref_path(user_ref_path)
            safe_plan = self.safety_planner.plan(initial_state, None)

            # If ILQR cannot find a safety plan, brake - or something else, we can change this
            # if safe_plan is None:
            #     safe_command = AckermannDriveStamped()
            #     safe_command.drive.speed = 0.0
            #     safe_command.drive.steering_angle = self.last_steering_angle
            #     self.get_logger().info(f"no safe plan")
            #     return safe_command
            
            plan_status = safe_plan['status']
            if plan_status == -1:
                safe_command = AckermannDriveStamped()
                safe_command.drive.speed = 0.0
                safe_command.drive.steering_angle = self.last_steering_angle
                self.get_logger().info(
                    f"safe planning failed"
                )
                return safe_command

            # Get cost of planned trajectory
            path_refs, obs_refs = self.safety_planner.get_references(safe_plan['trajectory'])
            safe_plan_cost = self.safety_planner.cost.get_traj_cost(safe_plan['trajectory'], safe_plan['controls'], path_refs, obs_refs)

            # debugging!!
            if safe_plan_cost > 400:

                self.get_logger().info(
                    f"Safe plan cost too high"
                )
                safe_command = AckermannDriveStamped()
                safe_command.drive.speed = 0.0
                safe_command.drive.steering_angle = self.last_steering_angle
                return safe_command

            # ILQR controls use [accel, steering_rate].
            # /drive needs [speed, steering_angle].
            safe_control = safe_plan['controls'][:, 0]
            safe_accel = safe_control[0]
            safe_steering_rate = safe_control[1]

            safe_speed = initial_state[2] + safe_accel * self.safety_planner.dt    # convert from acc to speed
            safe_steering_angle = initial_state[4] + safe_steering_rate * self.safety_planner.dt # convert from steering rate to angle
            #print(f"Steering rate {safe_steering_angle}. Initial steering angle: {initial_state[4]}")

            self.get_logger().info(
                    f"Safe cost: {safe_plan_cost} speed {safe_speed}"
                )

            # clamp speed and angle
            safe_speed = max(0.0, min(1.0, safe_speed))
            safe_steering_angle = max(-0.34, min(0.34, safe_steering_angle))

            safe_command = AckermannDriveStamped()
            safe_command.drive.speed = safe_speed
            safe_command.drive.steering_angle = safe_steering_angle
            # safe_command.drive.acceleration = float(safe_accel)

            # print("Publishing safe command")
            # self.get_logger().info(
            #     f"publishing safe command"
            # )
            #print(f"Safe command: Drive speed: {safe_speed}. Steering angle: {safe_steering_angle}")

            # self.get_logger().info(f"safe command: {safe_command}")
            return safe_command
      

def main(args=None):
    rclpy.init(args=args)
    node = SafetyFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()