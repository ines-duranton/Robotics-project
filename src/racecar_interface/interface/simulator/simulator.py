#ros2 packages
import rclpy
import rclpy.service
from rclpy.node import Node
from nav_msgs.msg import Odometry
from racecar_msgs.msg import ServoMsg
from scipy.spatial.transform import Rotation as R
from rclpy.clock import Clock
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from rcl_interfaces.msg import SetParametersResult

from racecar_interface.srv import Reset    #create new service file

#python packages
import threading
import queue
import numpy as np

#self defined packages
from .realtime_buffer import RealtimeBuffer
from .dynamics import Bicycle4D

"""
Simulator for central Car
"""
class Simulator(Node):
    def __init__(self):
        super().__init__("simulator_node")


        #Declare Parameters
        self.declare_parameter('control_topic', '/control')
        self.declare_parameter('odom_topic', '/slam_pose')
        self.declare_parameter('pub_rate', 30)
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_yaw', 0.0)
        self.declare_parameter('reset_service', '/simulation/reset')

        #values
        control_topic = self.get_parameter('control_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.pub_rate = self.get_parameter('pub_rate').value
        init_x = self.get_parameter('init_x').value
        init_y = self.get_parameter('init_y').value
        init_yaw = self.get_parameter('init_yaw').value
        reset_srv_name = self.get_parameter('reset_service').value


        #Setting Parameter boundaries for noise and latency
        #to be displayed in a GUI in RVIZ
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
        self.update_lock = threading.Lock()


        #ego dynamics
        self.current_state = np.array([init_x, init_y, 0, init_yaw])        
        self.dyn = Bicycle4D(1.0/self.pub_rate)
        self.control_buffer = RealtimeBuffer()
        
        #pub / sub
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 1)
        self.control_sub = self.create_subscription(ServoMsg, control_topic, self.control_callback, 1)

        #no more dynamic reconfigure, so change using parameter callback
        self.add_on_set_parameters_callback(self._on_params)

        self.reset_srv = self.create_service(Reset, reset_srv_name, self.reset_cb)

        #Publish rate of simulation
        #threading.Thread(target=self.simulation_thread).start()
        self.timer = self.create_timer(1.0 / float(self.pub_rate), self.simulation_step)
        self.msg_queue = queue.Queue()

        #clock
        self.clock = Clock()




    
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
    
    def reset_cb(self, req, res):
        self.update_lock.acquire(blocking=True)
        self.current_state = np.array([req.x, req.y, 0, req.yaw])
        self.get_logger().info(f"ego state reset to {self.current_state}")
        self.update_lock.release()
        res.success = True

        return res
        
    def control_callback(self, msg):
        control = np.array([msg.throttle, msg.steer])
        self.control_buffer.writeFromNonRT(control)
        
    def simulation_step(self):
        self.update_lock.acquire()
        control = self.control_buffer.readFromRT()
        if control is not None:
            self.current_state = self.dyn.integrate(self.current_state, control, self.sigma)
        
        self.current_state[3] = np.arctan2(np.sin(self.current_state[3]), np.cos(self.current_state[3]))
        odom_msg = Odometry()
        odom_msg.header.stamp = self.clock.now().to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.pose.pose.position.x = self.current_state[0]
        odom_msg.pose.pose.position.y = self.current_state[1]
        odom_msg.pose.pose.position.z = 0.0
        r = R.from_rotvec(self.current_state[3] * np.array([0, 0, 1]))
        q = r.as_quat()
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        #storing velocity in twist
        odom_msg.twist.twist.linear.x = self.current_state[2]
        
        if self.reset_latency:
            print("Clearing latency Queue")
            self.msg_queue.queue.clear()
            self.reset_latency = False
            
        self.msg_queue.put(odom_msg)
        t_cur = self.get_clock().now().nanoseconds * 1e-9
        t_queue_top = self.msg_queue.queue[0].header.stamp 
        prev_time = t_queue_top.sec + t_queue_top.nanosec * 1e-9
        dt = t_cur - prev_time
        
        # simulate latency with delayed publishing
        if dt >=  self.latency:
            odom_msg = self.msg_queue.get()
            self.odom_pub.publish(odom_msg)
        
        # latency.sleep()
        # self.odom_pub.publish(odom_msg)
        self.update_lock.release()