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

from ece346.FinalProject.ILQR_Example.ref_path import RefPath
from ece346.FinalProject.ILQR_Example.ilqr import ILQR
from ece346.Lab3.scripts.utils.static_obstacle import get_obstacle_vertices

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

        self.declare_parameter('ilqr_params_file', os.path.join(self.package, "config", "final_truck_ilqr.yaml"))

        # For routing path
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('routing_path_topic', '/Routing/Path')

        self.declare_parameter('projection_dt')
        self.declare_parameter('total_soft_threshold') # added to make it easy to change

        # ---- TODO(Task 1.2): read parameter values ----

        teleop_topic1 = self.get_parameter('teleop_topic').value     # AckermannDriveStamped
        odom_topic = self.get_parameter('odom_topic').value      # Odometry
        obs_topic = self.get_parameter('static_obs_topic').value        # MarkerArray
        drive_topic = self.get_parameter('drive_topic').value
        goal_topic = self.get_parameter('goal_topic').value # Pose
        routing_topic = self.get_parameter('routing_path_topic').value
        self.ilqr_params_file = self.get_parameter('ilqr_params_file').value

        self.publish_rate = self.get_parameter('publish_rate').value

        self.dt = self.get_parameter('projection_dt').value

        self.total_soft_threshold = self.get_parameter('total_soft_threshold').value
        self.safe_release_threshold = 0.5 * self.total_soft_threshold               # ==================== Do you wanna use something like this not to make the car jittery?
        self.safe_trigger = False # not 1 or 0 as i initially said in the notes

        self.teleop_current = None
        self.odom_current = None
        self.obs_stat_current = None
        self.goal_current = None
        self.goal_routing = None

        self.obstacle_list = []

        self.ref_path = None

        self.planner = ILQR(logger = self.get_logger(), config_file = self.ilqr_params_file)

        # ---- TODO(Task 1.3): create subscribers ----
        self.create_subscription(AckermannDriveStamped, teleop_topic1, self.teleop_cb, 1)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 1)
        self.create_subscription(MarkerArray, obs_topic, self.obs_stat_cb, 1)
        self.create_subscription(PoseStamped, goal_topic, self.goal_cb, 1)
        self.create_subscription(PathMsg, routing_topic, self.routing_cb, 1)

        # ---- TODO(Task 1.4): create the publisher ----
        self.pub = self.create_publisher(AckermannDriveStamped, drive_topic, 1)

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
    
    def obs_stat_cb(self, msg: MarkerArray):
        self.obs_stat_current = msg

    def goal_cb(self, msg: PoseStamped):
        self.goal_current = msg

    def routing_cb(self, msg: PathMsg):
        self.goal_routing = msg


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


#         # This code is copied and then modified from traj_planner_example in order to estimate the current state
#         # check if there is new state available
#         u_queue = queue.Queue()

#         # initialize the control command
#         accel = -5.0
#         steer = 0.0
#         state_cur = None
#         policy = self.policy_buffer.readFromRT()
            
#         # take the latency of publish into the account
#         if self.simulation:
#             t_act = self.get_clock().now().nanoseconds * 1e-9
#         else:
#             self.update_lock.acquire()
#             t_act = (self.get_clock().now().nanoseconds * 1e-9) + self.latency 
#             self.update_lock.release()
        
#         if odom:
#             slam_time = odom.header.stamp
#             t_slam = slam_time.sec +  slam_time.nanosec * 1e-9
            
#             u = np.zeros(3)
#             u[-1] = t_slam
#             while not u_queue.empty() and u_queue.queue[0][-1] < t_slam:
#                 u = u_queue.get() # remove old control commands
            
#             # get the state from the odometry message
#             q = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, 
#                     odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
#             # get the heading angle from the quaternion
#             psi = R.from_quat(q).as_euler('xyz', degrees=False)[-1]
                
#             state_cur = np.array([
#                         odom.pose.pose.position.x,
#                         odom.pose.pose.position.y,
#                         odom.twist.twist.linear.x,
#                         psi,
#                         u[1]
#                     ])
               
#             # predict the current state use past control command
#             for i in range(u_queue.qsize()):
#                 u_next = u_queue.queue[i]
#                 dt = u_next[-1] - u[-1]
#                 state_cur = dyn_step(state_cur, u, dt)
#                 u = u_next
                    
#             # predict the cur state with the most recent control command
#             state_cur = dyn_step(state_cur, u, t_act - u[-1])
                
#             # update the state buffer for the planning thread (can prob take this out bc not using planning thread)
#             plan_state = np.append(state_cur, t_act)
#             self.plan_state_buffer.writeFromNonRT(plan_state)
    
#         # if there is no new state available, we do one step forward integration to predict the state
#         elif prev_state is not None:
#             t_prev = prev_u[-1]
#             dt = t_act - t_prev
#             # predict the state using the last control command is executed
#             state_cur = dyn_step(prev_state, prev_u, dt)

# # END COPIED CODE

        # This is a VERY rough outline of what we should be doing for the user ILQR trajectory without the overriding safety ILQR
        # It's based on calvin's recommendations, but def won't run and will need some debugging
        # the first step of debugging is done, no errors in the code show up, but the car has the weirdest behavior
        # if you set a goal and press on the acceleration for long enough, the car won't move at first and then run away from the screen

        # time_steps = 40 # Number of time steps in the ILQR (for now)
        # control_dim = 2

        # # Using /Routing/Path to set goal points
        # if goal is not None:
        #     x_goal, y_goal = goal.pose.position.x, goal.pose.position.y

        # Get current state and control actions from ROS topics
        intial_yaw = yaw_from_quat(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
        initial_x = odom.pose.pose.position.x
        initial_y = odom.pose.pose.position.y
        initial_v = max(0.0, odom.twist.twist.linear.x) # dunnot if fix, but we also do it in the dym step
        initial_steering_angle = teleop.drive.steering_angle
        initial_acc = teleop.drive.acceleration

        # Create np arrays for the state and control
        initial_state = np.array([initial_x, initial_y, initial_v, intial_yaw, initial_steering_angle])
        #initial_control = np.array([initial_acc, initial_steering_angle]) 
        # dyn_step uses u = [accel, steering_angle].

        # From bicyle5d
        # State: [x, y, v, psi, delta]
		# 	Control: [accel, omega]
		# 	dx_k = v_k cos(psi_k)
		# 	dy_k = v_k sin(psi_k)
		# 	dv_k = accel_k
		# 	dpsi_k = v_k tan(delta_k) / L
		# 	ddelta_k = omega_k
        
        # Roll out user trajectory for 10 time steps
        state_after_user_control = initial_state.copy() # do not mutate the state 
        for i in range(10):
            state_after_user_control = dyn_step(state_after_user_control, initial_control, self.dt)


        # Update obstacles

        obstacle_list = []

        # like in traj_planner_example obstacle callback
        for obs in obstacles.markers:
            _, vertices = get_obstacle_vertices(obs)
            obstacle_list.append(vertices)
        
        # Set up ILQR planner (this line is taken from traj_planner_example.py)
        self.planner = ILQR(logger = self.get_logger(), config_file = self.ilqr_params_file)
        user_cost = 0 #just for the test
        if routing:
            user_ref_path = path_callback(routing) # Centerline routing path
            if user_ref_path :
                self.planner.update_ref_path(user_ref_path) # Update reference path
                user_plan = self.planner.plan(state_after_user_control, None) # Plan with ILQR based on 10 steps of user control
                # Get cost of planned trajectory
                path_refs, obs_refs = self.planner.get_references(user_plan['trajectory'])
                user_cost = self.planner.cost.get_traj_cost(user_plan['trajectory'], user_plan['controls'], path_refs, obs_refs)
                print('cost :', user_cost)

        # If below (very arbitary) threshold, publish user control
        if user_cost < 100:
            #print("Running teleop because user plan cost is low")
            return teleop
        else: # Fix later
            return None


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