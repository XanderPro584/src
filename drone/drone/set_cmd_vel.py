#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import time

# import qos profile dependencies
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Twist
# import command velocity
from drone_interfaces.srv import CommandVelocity
# import local frame offsets
from drone_interfaces.msg import LocalFrameOffsets
# import float64
from example_interfaces.msg import Float64

# import math and pi
from math import atan2, pow, sqrt, degrees, radians, sin, cos, pi
import numpy as np


class SetCmdVelNode(Node): 
    def __init__(self):
        super().__init__("set_cmd_vel") 
        self.get_logger().info("Set Cmd Vel Node has been started")
        self.command_velocity = CommandVelocity.Request()
        self.local_frame_offsets = LocalFrameOffsets()
        self.current_heading = Float64(data=0.0)
        self.rotation_theta = 0.0
        self.translation_x = 0.0
        self.translation_y = 0.0
        self.max_linear_velocity = 13.0
        self.max_angular_velocity = 60.0
        self.publishing_cmd_vel = False

        # Volatile Quality of Service profile
        self.volatile_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist, "mavros/setpoint_velocity/cmd_vel_unstamped", 10, callback_group=MutuallyExclusiveCallbackGroup())
        # Local frame offset subscriber
        self.local_frame_offsets_sub = self.create_subscription(
            LocalFrameOffsets, "/local_frame_offsets", self.local_frame_offsets_cb, self.volatile_qos_profile)
        self.current_heading_sub = self.create_subscription(
            Float64, "/local_heading", self.current_heading_cb, self.volatile_qos_profile)
        
        # Command velocity server
        self.cmd_vel_server = self.create_service(
            CommandVelocity, "/set_cmd_vel", self.cmd_vel_cb, callback_group=MutuallyExclusiveCallbackGroup())
        self.pub_vel_timer = self.create_timer(
            0.1, self.publish_cmd_vel, callback_group=MutuallyExclusiveCallbackGroup())
        
    def cmd_vel_cb(self, request, response):
        # Check that the linear and angular velocity are within the limits
        response = CommandVelocity.Response()

        if self.check_if_too_fast(request):
            self.get_logger().warn("Linear or Angular velocity is too high")
            response.success = False
            response.message = "Linear or Angular velocity is too high"
            return response
        else:
            self.command_velocity = request
            # log linear_x of request
            self.get_logger().info(f"Linear x of request: {request.linear_x}")
            # log the value of publish_cmd_vel
            self.get_logger().info(f"Publishing cmd vel: {self.publishing_cmd_vel}")
            if not self.publishing_cmd_vel:
                self.publishing_cmd_vel = True
                self.get_logger().info("Setting publishing to true")
            # Check if the command velocity is zero
            if self.check_if_zero(request):
                self.publish_cmd_vel()
                self.publishing_cmd_vel = False
            self.get_logger().info("Command velocity has been set")
            response.success = True
            response.message = "Command velocity has been set"
            return response
        
    def check_if_too_fast(self, request_cmd_vel):
        # Check if the linear and angular velocity are within the limits
        if abs(request_cmd_vel.linear_x) > self.max_linear_velocity \
            or abs(request_cmd_vel.linear_y) > self.max_linear_velocity \
            or abs(request_cmd_vel.linear_z) > self.max_linear_velocity \
            or abs(request_cmd_vel.angular_x) > self.max_angular_velocity \
            or abs(request_cmd_vel.angular_y) > self.max_angular_velocity \
            or abs(request_cmd_vel.angular_z) > self.max_angular_velocity:
            return True
        else:
            return False
        
    def check_if_zero(self, request_cmd_vel):
        # Check if the command velocity is zero
        if request_cmd_vel.angular_x == 0.0 \
            and request_cmd_vel.angular_y == 0.0 \
            and request_cmd_vel.angular_z == 0.0 \
            and request_cmd_vel.linear_x == 0.0 \
            and request_cmd_vel.linear_y == 0.0 \
            and request_cmd_vel.linear_z == 0.0:
            self.get_logger().info("Command velocity is zero")
            return True
        else:
            return False
        
    def publish_cmd_vel(self):
        # Publish the command velocity
        if self.publishing_cmd_vel:
            self.get_logger().info("Publishing command velocity")
            original_cmd_vel = self.get_original_cmd_vel()
            self.cmd_vel_pub.publish(original_cmd_vel)

        else: 
            # self.get_logger().info("Command velocity is zero")
            pass

    def get_original_cmd_vel(self):
        # Get the original command velocity
        original_cmd_vel = Twist()
        original_x_y = self.transform_point(self.command_velocity.linear_x, self.command_velocity.linear_y)
        original_cmd_vel.linear.x = original_x_y[0]
        original_cmd_vel.linear.y = original_x_y[1]
        original_cmd_vel.linear.z = self.command_velocity.linear_z
        original_cmd_vel.angular.x = radians(self.command_velocity.angular_x)
        original_cmd_vel.angular.y = radians(self.command_velocity.angular_y)
        original_cmd_vel.angular.z = radians(self.command_velocity.angular_z)

        return original_cmd_vel

    def transform_point(self, x, y):
        # Apply rotation first and then translation
        theta = radians(self.rotation_theta + self.current_heading - 90)
        rotation_matrix = np.array([[cos(theta), -sin(theta)],
                                    [sin(theta), cos(theta)]])
        rotated_point = np.dot(rotation_matrix, np.array([x, y]))

        translated_point = rotated_point + np.array([self.translation_x, self.translation_y])

        return translated_point
    
    def local_frame_offsets_cb(self, msg):
        self.local_frame_offsets = msg
        self.rotation_theta = self.local_frame_offsets.rotation_theta

    def current_heading_cb(self, msg):
        self.current_heading = msg.data

        


def main(args=None):
    rclpy.init(args=args)
    node = SetCmdVelNode() 
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()