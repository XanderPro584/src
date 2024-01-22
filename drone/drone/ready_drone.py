#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import time
from functools import partial

from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import State

from drone_interfaces.action import ReadyDrone
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle

class ReadyDroneNode(Node): 
    def __init__(self):
        super().__init__("ready_drone") 
        self.get_logger().info("Ready Drone Node has been started")

        self.volatile_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.ready_drone_cb_group = ReentrantCallbackGroup()

        self.local_pose = Point()
        self.local_heading = Float64()
        self.current_state = State()

        self.is_armed = False
        self.is_takeoff = False

        self.ready_drone = ActionServer(
            self,
            ReadyDrone,
            "ready_drone",
            goal_callback=self.goal_callback,
            execute_callback=self.execute_callback,
            callback_group=self.ready_drone_cb_group
        )

        self.arm_client = self.create_client(
            CommandBool, "/mavros/cmd/arming", callback_group=MutuallyExclusiveCallbackGroup())
        self.takeoff_client = self.create_client(
            CommandTOL, "/mavros/cmd/takeoff", callback_group=MutuallyExclusiveCallbackGroup())
        self.initialize_drone_pose_client = self.create_client(
            CommandBool, "/initialize_drone_pose", callback_group=MutuallyExclusiveCallbackGroup())
        
        self.state_sub = self.create_subscription(
            State, "/mavros/state", self.state_cb, 10)


    def goal_callback(self, goal_request: ReadyDrone.Goal):
        self.get_logger().info(f"Received a goal request: {goal_request}")
        return GoalResponse.ACCEPT
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Executing goal...")

        feedback = ReadyDrone.Feedback()
        result = ReadyDrone.Result()
        altitude = goal_handle.request.altitude

        feedback.message = "waiting guided"
        goal_handle.publish_feedback(feedback)
        self.wait_for_guided()

        feedback.message = "initializing drone pose"
        goal_handle.publish_feedback(feedback)
        drone_pose_is_initialized = self.initialize_drone_pose()

        if drone_pose_is_initialized:
            self.get_logger().info("Drone pose is initialized")
            feedback.message = "arming drone"
            goal_handle.publish_feedback(feedback)
            self.is_armed = self.arm_drone()

        if self.is_armed:
            time.sleep(10)
            feedback.message = "taking off"
            goal_handle.publish_feedback(feedback)
            self.is_takeoff = self.takeoff_drone(altitude)
        
        if self.is_takeoff:
            self.get_logger().info("Drone is taking off")
            time.sleep(3)
            result.success = True
            goal_handle.succeed()
            return result
        else:
            result.success = False
            self.get_logger().info("Drone failed to take off. Aborting")
            goal_handle.abort()
            return result


    def initialize_drone_pose(self):
        self.get_logger().info("Initializing drone_pose")
        self.initialize_drone_pose_client.wait_for_service()
        request = CommandBool.Request()
        request.value = True
        result = self.initialize_drone_pose_client.call(request)
        return result.success

    def wait_for_guided(self):
        self.get_logger().info("Waiting for guided mode")
        while not self.current_state.mode == "GUIDED":
            time.sleep(0.1)
        self.get_logger().info("Guided mode reached")

    def arm_drone(self):
        self.get_logger().info("Arming drone")
        self.arm_client.wait_for_service()
        request = CommandBool.Request()
        request.value = True
        result = self.arm_client.call(request)
        return result.success
    
    def takeoff_drone(self, altitude):
        self.get_logger().info("Taking off drone")
        self.takeoff_client.wait_for_service()
        request = CommandTOL.Request()
        request.altitude = altitude
        result = self.takeoff_client.call(request)
        return result.success

    def state_cb(self, data):
        self.current_state = data
        
        

def main(args=None):
    rclpy.init(args=args)
    node = ReadyDroneNode() 
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()