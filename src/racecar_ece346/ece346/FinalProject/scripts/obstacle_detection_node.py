#!/usr/bin/env python3
"""
Static obstacle detection from AprilTag detections.
Subscribes to SLAM pose and tag detections, transforms tag positions
to world frame, and publishes detected obstacles as MarkerArray.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import numpy as np

from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from racecar_msgs.msg import AprilTagDetectionArray

import message_filters
from scipy.spatial.transform import Rotation as Rot


def pose2T(pose):
    '''Convert a geometry_msgs/Pose to a 4x4 transformation matrix.'''
    T = np.eye(4)
    q = [pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w]
    T[:3, :3] = Rot.from_quat(q).as_matrix()
    T[0, 3] = pose.position.x
    T[1, 3] = pose.position.y
    T[2, 3] = pose.position.z
    return T


def quaternion_from_matrix(T):
    '''Extract quaternion (x, y, z, w) from a 4x4 transformation matrix.'''
    return Rot.from_matrix(T[:3, :3]).as_quat()


class StaticObstacleDetector(Node):
    def __init__(self):
        super().__init__('static_obstacle_detection_node')

        self.declare_parameter('odom_topic', '/SLAM/Pose')
        self.declare_parameter('static_tag_topic', '/SLAM/Tag_Detections_Dynamic')
        self.declare_parameter('static_obs_size', 0.2)
        self.declare_parameter('static_obs_topic', '/Obstacles/Static')

        odom_topic = self.get_parameter('odom_topic').value
        tag_topic = self.get_parameter('static_tag_topic').value
        self.static_obs_size = self.get_parameter('static_obs_size').value
        obs_topic = self.get_parameter('static_obs_topic').value

        # Camera to rear axle transform (truck-specific)
        self.T_rob2cam = np.array([
            [1.0, 0.0, 0.0, -0.357],
            [0.0, 1.0, 0.0, -0.06],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
        self.T_cam2rob = np.linalg.inv(self.T_rob2cam)

        # Tag is on top of obstacle cube
        self.T_obs2tag = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, -self.static_obs_size / 2.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

        self.obs_pub = self.create_publisher(MarkerArray, obs_topic, 1)

        # Synchronized subscribers for pose + tag detections
        pose_sub = message_filters.Subscriber(self, Odometry, odom_topic)
        tag_sub = message_filters.Subscriber(
            self, AprilTagDetectionArray, tag_topic
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [pose_sub, tag_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.detect_obs)

        self.get_logger().info(
            f"Obstacle detection ready. Listening on {odom_topic} + {tag_topic}"
        )

    def detect_obs(self, odom_msg, tag_list):
        detected_tag = []
        static_obs_msg = MarkerArray()

        T_rob2world = pose2T(odom_msg.pose.pose)
        T_cam2world = T_rob2world @ self.T_cam2rob

        for tag in tag_list.detections:
            if tag.id in detected_tag:
                continue
            detected_tag.append(tag.id)

            T_tag2cam = pose2T(tag.pose)
            T_tag2world = T_cam2world @ T_tag2cam
            T_obs2world = T_tag2world @ self.T_obs2tag

            marker = Marker()
            marker.header = odom_msg.header
            marker.ns = 'static_obs'
            marker.id = tag.id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = T_obs2world[0, 3]
            marker.pose.position.y = T_obs2world[1, 3]
            marker.pose.position.z = T_obs2world[2, 3]

            q = quaternion_from_matrix(T_obs2world)
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]

            marker.scale.x = self.static_obs_size
            marker.scale.y = self.static_obs_size
            marker.scale.z = self.static_obs_size

            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 153.0 / 255.0
            marker.color.a = 0.8

            static_obs_msg.markers.append(marker)

        self.obs_pub.publish(static_obs_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StaticObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
