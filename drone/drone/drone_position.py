#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import numpy as np
from math import atan2, pow, sqrt, degrees, radians, sin, cos

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from example_interfaces.msg import Float64, Bool
from drone_interfaces.msg import LocalFrameOffsets

class DronePositionNode(LifecycleNode): 
    def __init__(self):
        super().__init__("drone_position") 
        self.get_logger().info("Drone Position Node has been started")

        self.ready_to_initialize_pub = self.create_publisher(
            Bool, "/ready_to_activate", qos_profile=10)
        
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring')

        self.frame_initialized = False
        self.current_pose_g = Odometry()
        self.current_heading_g = 0.0
        self.rotation_theta = 0.0
        self.gps_fix = False
        self.ready_to_initialize = False
        self.current_pose_list = []
        self.local_frame_offsets = LocalFrameOffsets()



        # Volatile Quality of Service profile
        self.volatile_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # set up publishers and subscribers
        # Check for GPS fix, this message is not used for anything else
        self.gps_pos_check = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.gps_pos_check_cb, qos_profile=self.volatile_qos_profile) 

        self.local_position_pub = self.create_lifecycle_publisher(
            Point, "/local_position", qos_profile=self.volatile_qos_profile)
        
        self.current_heading_pub = self.create_lifecycle_publisher(
            Float64, "/local_heading", qos_profile=self.volatile_qos_profile)

        self.currentPos = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.pose_cb, qos_profile=self.volatile_qos_profile,)
        
        self.local_frame_offsets_pub = self.create_lifecycle_publisher(
            LocalFrameOffsets, "/local_frame_offsets", qos_profile=self.volatile_qos_profile)
        
        self.get_logger().info('Getting ready to initialize local frame ...')
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up')
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating')
        self.get_logger().info('Initializing local frame ...')
        self.initialize_local_frame()
        if not self.frame_initialized:
            return TransitionCallbackReturn.ERROR
        else: 
            self.waypoint_server_is_activated = True
            return super().on_activate(state)
        
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating')
        self.current_heading_g = 0.0
        self.current_pose_list = []
        self.gps_fix = False
        self.frame_initialized = False
        return super().on_deactivate(state)
    
    def pose_cb(self, msg):
        self.current_pose_g = msg

        if self.gps_fix:
            if len(self.current_pose_list) >= 10:
                if self.ready_to_initialize == False:
                    self.get_logger().info("Ready to initialize")
                    self.ready_to_initialize = True

                self.current_pose_list.pop(0)
                self.ready_to_initialize_pub.publish(Bool(data=True))


            self.current_pose_list.append(self.current_pose_g)

        q0, q1, q2, q3 = (
            self.current_pose_g.pose.orientation.w,
            self.current_pose_g.pose.orientation.x,
            self.current_pose_g.pose.orientation.y,
            self.current_pose_g.pose.orientation.z,
        )
        psi = atan2((2 * (q0 * q3 + q1 * q2)),
                    (1 - 2 * (pow(q2, 2) + pow(q3, 2))))
        
        self.current_heading_g = degrees(psi) - self.rotation_theta
        if self.frame_initialized:
            self.current_heading_pub.publish(Float64(data=self.current_heading_g))
            local_position = self.get_local_position()
            self.local_position_pub.publish(local_position)
            self.local_frame_offsets.translation_x = self.translation_x
            self.local_frame_offsets.translation_y = self.translation_y
            self.local_frame_offsets.rotation_theta = self.rotation_theta
            self.local_frame_offsets_pub.publish(self.local_frame_offsets)

    def gps_pos_check_cb(self, msg):
        if self.gps_fix == False:
            self.gps_fix = True      
            self.get_logger().info("GPS fix acquired")  

    def get_local_position(self):
        x = self.current_pose_g.pose.position.x
        y = self.current_pose_g.pose.position.y
        z = self.current_pose_g.pose.position.z
        
        rotated_point = self.inverse_transform_point(x, y)

        position = Point()
        position.x = rotated_point[0]
        position.y = rotated_point[1]
        position.z = z

        return position
    
    def transform_point(self, x, y):
        # Apply rotation first and then translation
        theta = radians(self.rotation_theta - 90)
        rotation_matrix = np.array([[cos(theta), -sin(theta)],
                                    [sin(theta), cos(theta)]])
        rotated_point = np.dot(rotation_matrix, np.array([x, y]))

        translated_point = rotated_point + np.array([self.translation_x, self.translation_y])

        return translated_point
    
    def inverse_transform_point(self, x, y):
        # Apply inverse translation first and then inverse rotation
        inverse_translated_point = np.array([x, y]) - np.array([self.translation_x, self.translation_y])

        theta = radians(-(self.rotation_theta - 90))
        inverse_rotation_matrix = np.array([[cos(theta), -sin(theta)],
                                           [sin(theta), cos(theta)]])
        inverse_rotated_point = np.dot(inverse_rotation_matrix, inverse_translated_point)

        return inverse_rotated_point
    
    def initialize_local_frame(self):
        # self.set_homepoint()
        self.rotation_theta = 0.0
        self.translation_x = 0.0
        self.translation_y = 0.0

        if len(self.current_pose_list) == 10:
            for previous_pose in self.current_pose_list:
                q0, q1, q2, q3 = (
                    previous_pose.pose.orientation.w,
                    previous_pose.pose.orientation.x,
                    previous_pose.pose.orientation.y,
                    previous_pose.pose.orientation.z,
                )

                psi = atan2((2 * (q0 * q3 + q1 * q2)),
                            (1 - 2 * (pow(q2, 2) + pow(q3, 2))))

                self.rotation_theta += degrees(psi)

                self.translation_x += previous_pose.pose.position.x
                self.translation_y += previous_pose.pose.position.y

            self.rotation_theta /= 10.0
            self.translation_x /= 10.0
            self.translation_y /= 10.0

            self.get_logger().info("coordinate offset set")        
            self.get_logger().info("The X-Axis is facing: {}".format(self.rotation_theta))
            self.get_logger().info("The coordinate offset is: ({}, {})".format(self.translation_x, self.translation_y))
            self.frame_initialized = True

        else:
            self.get_logger().info("Not enough data to initialize local frame")
            self.frame_initialized = False


def main(args=None):
    rclpy.init(args=args)
    node = DronePositionNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()