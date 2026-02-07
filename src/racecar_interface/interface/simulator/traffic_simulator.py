import rclpy
import rclpy.service
from rclpy.node import Node
import pickle
import numpy as np
from .dynamics import Bicycle4D
from .ref_path import RefPath
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
from racecar_msgs.msg import OdometryArray
from scipy.spatial.transform import Rotation as R
import threading
from racecar_interface.srv import ResetObstacle
from ament_index_python.packages import get_package_share_directory
from pathlib import Path as FsPath
from lanelet2.projection import LocalCartesianProjector
import queue
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from rcl_interfaces.msg import SetParametersResult
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header
from rclpy.clock import Clock
from rclpy.time import Time

from routing.routing.lanelet_wrapper import LaneletWrapper

"""
Simulator for Adversarial cars / Objects
"""
class TrafficSimulator(Node):
    def __init__(self):
        super().__init__("traffic_simulation_node")

        self.declare_read_parameters()
        
        #latency / noise
        self.sigma = np.array([self.get_parameter('throttle_noise_sigma').value,
                               self.get_parameter('steer_noise_sigma').value])
        self.latency = self.get_parameter('latency').value
        self.reset_latency = False
        self.update_lock = threading.Lock()
        self.clock = Clock()
        
        self.dyn = Bicycle4D(1.0/self.pub_rate)
        
        self.add_on_set_parameters_callback(self._on_params)
        self.reset_srv = self.create_service(ResetObstacle, self.reset_srv_name, self.reset_cb)        
        
        self.setup_publisher()

        threading.Thread(target=self.simulation_thread, daemon=True).start()

    def reset_cb(self, req, res):
        self.update_lock.acquire()
        self.num_static_obj = req.n
        self.static_obs_msg = self.create_static_obs()
        self.get_logger().info(f"Static Obstacle Reset")
        self.update_lock.release()
        res.success = True
        return res
        
    def declare_read_parameters(self):
        #declare parameters
        try:
            share_dir = FsPath(get_package_share_directory('racecar_routing'))
        except Exception as e:
            self.get_logger().error(f"Could not locate package 'racecar_routing' share dir: {e}")
            self.destroy_node()
            rclpy.shutdown()
            return
        
        
        self.declare_parameter('~num_dyn_obs', 1)
        self.declare_parameter('~num_static_obs', 1)
        self.declare_parameter('~static_obs_size', 0.2)
        self.declare_parameter('static_obs_topic', '/Obstacles/Static')
        self.declare_parameter('dyn_obs_topic', '/Obstacles/Dynamic')
        self.declare_parameter('pub_rate', 30)
        self.declare_parameter('static_obs_location', '')
        self.declare_parameter('map_file', str(share_dir / 'maps' / 'track.osm'))
        self.declare_parameter('reset_obstacle_service', '/simulation/reset_static_obstacles')

        self.map_file = self.get_parameter('map_file').value
        self.reset_srv_name = self.get_parameter('reset_obstacle_service').value   
        self.num_dyn_obj = self.get_parameter('~num_dyn_obs').value
        self.num_static_obj = self.get_parameter('~num_static_obs').value
        self.static_obs_size = self.get_parameter('~static_obs_size').value
        self.static_obs_topic = self.get_parameter('static_obs_topic').value
        self.dyn_obs_topic = self.get_parameter('dyn_obs_topic').value
        self.pub_rate = self.get_parameter('pub_rate').value
        self.static_obs_location = self.get_parameter('static_obs_location').value
        
        self.lane_change_cost = 1.0
        self.lanelet_wrapper = LaneletWrapper(self.map_file, self)

        if self.static_obs_location != '':
            self.static_obs_msg = self.load_static_obs(self.static_obs_location)
        else:
            self.get_logger().info("creating new static obstacles")
            self.static_obs_msg = self.create_static_obs()

        #parameters that will be changed via GUI
        self.declare_parameter('throttle_noise_sigma', 0.0, descriptor= 
                               ParameterDescriptor(
                                   name = 'throttle_noise_sigma',
                                   type = 3, #double
                                   description = 'Noise std',
                                   read_only = False,
                                   floating_point_range=[FloatingPointRange(
                                        from_value=0.0,
                                        to_value=5.0,
                                        step=0.0
                                    )] 
                               ))
        
        self.declare_parameter('steer_noise_sigma', 0.0, descriptor= 
                               ParameterDescriptor(
                                   name = 'steer_noise_sigma',
                                   type = 3, #double    

                                   description = 'Noise std',
                                   read_only = False,
                                   floating_point_range=[FloatingPointRange(
                                        from_value=0.0,
                                        to_value=5.0,
                                        step=0.0
                                    )] 
                               ))
        self.declare_parameter('latency', 0.0, descriptor= 
                               ParameterDescriptor(
                                   name = 'latency',
                                   type = 3, #double
                                   description = 'state latency',
                                   read_only = False,
                                   floating_point_range=[FloatingPointRange(
                                        from_value=0.0,
                                        to_value=1.0,
                                        step=0.0
                                    )] 
                               ))
        
        #latency / noise
        self.sigma = np.array([self.get_parameter('throttle_noise_sigma').value,
                               self.get_parameter('steer_noise_sigma').value])
        self.latency = self.get_parameter('latency').value
        self.reset_latency = False
            
    def setup_publisher(self):
        self.static_obs_pub = self.create_publisher(MarkerArray, self.static_obs_topic, 1)
        self.dyn_obs_pub = self.create_publisher(OdometryArray, self.dyn_obs_topic, 1)
        

    def _on_params(self, params):
        self.update_lock.acquire(blocking=True)
        prev_latency = self.latency
        for p in params:
            if p.name == 'throttle_noise_sigma':
                self.sigma[0] = p.value
            if p.name == 'steer_noise_sigma':
                self.sigma[1] = p.value
            if p.name == 'latency':
                self.latency = p.value
        self.reset_latency = self.latency != prev_latency
        self.get_logger().info(f"Simulation Noise Updated to {self.sigma}. Latency Updated to {self.latency} s")
        self.update_lock.release()
        return SetParametersResult(successful=True) 
        
    def simulation_thread(self):
        print("enter simulation thread")
        dyn_obs_pose = {}
        for i in range(self.num_dyn_obj):
            pose = self.lanelet_wrapper.get_random_waypoint()
            x,y,psi = pose
            #[x, y, v , psi]
            state = np.array([x, y, 0, psi])
            ref_path, v_ref = self.gen_ref_path(pose)
            dyn_obs_pose[i] = {'state': state, 'path': ref_path, 'v_ref': v_ref}
        
        rate = self.create_rate(self.pub_rate)
        msg_queue = queue.Queue()
        
        while rclpy.ok():
            odom_array_msg = OdometryArray()
            
            header = Header()
            header.frame_id = 'map'
            header.stamp = self.clock.now().to_msg()
            odom_array_msg.header = header
            
            for i in range(self.num_dyn_obj):
                state = dyn_obs_pose[i]['state']
                ref_path = dyn_obs_pose[i]['path']
                v_ref = dyn_obs_pose[i]['v_ref']
                _, _, s = ref_path.get_closest_pts(state[:2])
                
                # replan randomly
                if (1-s)*ref_path.length < state[2] or np.random.uniform()<0.03:
                    pose = np.array([state[0], state[1], state[3]])
                    ref_path, v_ref = self.gen_ref_path(pose)

                    dyn_obs_pose[i]['path'] = ref_path
                    dyn_obs_pose[i]['v_ref'] = v_ref
                    _, _, s = ref_path.get_closest_pts(state[:2])
                
                # calculate control
                # look ahead 1 second
                look_ahead, _ = ref_path.interp(s*ref_path.length + 0.5)
                look_ahead_x = look_ahead[0,0]
                look_ahead_y = look_ahead[1,0]
                
                # apply pure pursuit
                alpha = np.arctan2(look_ahead_y - state[1], look_ahead_x - state[0]) - state[3]
                ld = np.sqrt((look_ahead_x - state[0])**2 + (look_ahead_y - state[1])**2)
                steer = np.arctan2(2*0.257*np.sin(alpha), ld)
                throttle = (v_ref - state[2])*0.5
                
                state_next = self.dyn.integrate(state, np.array([throttle, steer]), self.sigma)
                dyn_obs_pose[i]['state'] = state_next
                
                # create message
                odom_msg = Odometry()
                odom_msg.header = header
                odom_msg.pose.pose.position.x = state_next[0]
                odom_msg.pose.pose.position.y = state_next[1]
                odom_msg.pose.pose.position.z = 0.0

                r = R.from_rotvec(np.array((0, 0, 1)) * state_next[3])
                q  = r.as_quat()

                odom_msg.pose.pose.orientation.x = q[0]
                odom_msg.pose.pose.orientation.y = q[1]
                odom_msg.pose.pose.orientation.z = q[2]
                odom_msg.pose.pose.orientation.w = q[3]
                
                odom_msg.twist.twist.linear.x = state_next[2]
                odom_array_msg.odometry_array.append(odom_msg)
                
            if self.reset_latency:
                print("Clearing Queue")
                msg_queue.queue.clear()
                self.reset_latency = False
                
            msg_queue.put(odom_array_msg)
            t_cur = self.get_clock().now().nanoseconds * 1e-9
            t_queue_top = msg_queue.queue[0].header.stamp 
            prev_time = t_queue_top.sec + t_queue_top.nanosec * 1e-9
            dt = t_cur - prev_time
            
            # simulate latency with delayed publishing
            if dt >=  self.latency:
                dyn_msg_to_pub = msg_queue.get()
                self.update_lock.acquire()
                static_msg_to_pub = self.static_obs_msg
                for obs in static_msg_to_pub.markers:
                    obs.header = dyn_msg_to_pub.header

                self.static_obs_pub.publish(static_msg_to_pub)
                self.update_lock.release()

                self.dyn_obs_pub.publish(dyn_msg_to_pub)
                
            rate.sleep()
            
    def gen_ref_path(self, pose):
        path = None
        while path is None:
            goal = self.lanelet_wrapper.get_random_waypoint()
            path = self.lanelet_wrapper.get_shortest_path(pose, goal, True, True, False)
        v_ref = np.random.uniform(0, 1.0) 
        path = np.array(path)
        return RefPath(path[:, :2].T, 0.5, 0.5, 1, True), v_ref
    
    def load_static_obs(self, locations):
        # Setup static obstacles
        static_obs_msg = MarkerArray()
        for i, xy in enumerate(locations):
            # randomly sample a pose
            psi = np.random.uniform(-np.pi, np.pi) # random heading
            
            marker = Marker()   
            marker.ns = 'static_obs'
            marker.id = i
            marker.type = 1 # cube
            marker.action = 0 # add
            
            # pose of the marker
            marker.pose.position.x = xy[0]
            marker.pose.position.y = xy[1]
            marker.pose.position.z = self.static_obs_size/2.0
            
            r = R.from_rotvec(np.array((0, 0, 1)) * psi)
            q  = r.as_quat()

            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
            
            marker.scale.x = self.static_obs_size
            marker.scale.y = self.static_obs_size
            marker.scale.z = self.static_obs_size
            
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 153/255.0
            marker.color.a = 0.8
            
            time = 1.5 / self.pub_rate
            seconds = int(time)
            nanoseconds = int((time - seconds) * 1e9) 
            marker.lifetime = Duration(sec = seconds, nanosec = nanoseconds)
            
            static_obs_msg.markers.append(marker)
        return static_obs_msg
    
        
            
    def create_static_obs(self):
        print(f"create {self.num_static_obj} static obstacles")
        # Setup static obstacles
        static_obs_msg = MarkerArray()
        for i in range(self.num_static_obj):
            # randomly sample a pose
            x, y, __ = self.lanelet_wrapper.get_random_waypoint(with_neighbors=True)
            psi = np.random.uniform(-np.pi, np.pi) # random heading
            
            marker = Marker()   
            marker.ns = 'static_obs'
            marker.id = i
            marker.type = 1 # cube
            marker.action = 0 # add
            
            # pose of the marker
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = self.static_obs_size/2.0
            
            r = R.from_rotvec(np.array((0, 0, 1)) * psi)
            q  = r.as_quat()
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
            
            marker.scale.x = self.static_obs_size
            marker.scale.y = self.static_obs_size
            marker.scale.z = self.static_obs_size
            
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 153/255.0
            marker.color.a = 0.8
            
            time = 1.5 / self.pub_rate
            seconds = int(time)
            nanoseconds = int((time - seconds) * 1e9) 
            marker.lifetime = Duration(sec = seconds, nanosec = nanoseconds)
            
            static_obs_msg.markers.append(marker)
        return static_obs_msg
        