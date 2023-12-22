#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

# import multithreading and reentrantcallbackgroup
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading


from example_interfaces.msg import Bool
from mavros_msgs.srv import CommandBool


class DroneLifecycleNodeManager(Node):
    def __init__(self):
        super().__init__("lifecycle_manager")
        self.get_logger().info("Drone Lifecycle Manager has been started")
        self.drone_pose_is_activated = False

        self.drone_pose_client = self.create_client(
            ChangeState, "/drone_position/change_state", callback_group=ReentrantCallbackGroup())
        
        self.initialize_drone_pose_server = self.create_service(
            CommandBool, "/initialize_drone_pose", self.initialize_drone_pose_cb, callback_group=MutuallyExclusiveCallbackGroup())
        
        self.subscriber_callback_event = threading.Event()

    def change_state(self, transition: Transition, client):
        client.wait_for_service()
        request = ChangeState.Request()
        request.transition = transition
        client.call(request)
        # rclpy.spin_until_future_complete(self, future)

    def configure_(self, client):
        # Unconfigured to Inactive
        self.get_logger().info("Trying to switch to configuring")
        transition = Transition()
        transition.id = Transition.TRANSITION_CONFIGURE
        transition.label = "configure"
        self.change_state(transition, client)
        self.get_logger().info("Configuring OK, now inactive")

    def activate_(self, client):
        # Inactive to Active
        self.get_logger().info("Trying to switch to activating")
        transition = Transition()
        transition.id = Transition.TRANSITION_ACTIVATE
        transition.label = "activate"
        self.change_state(transition, client)
        self.get_logger().info("Activating OK, now active")

    def deactivate_(self, client):
        # Active to Inactive
        self.get_logger().info("Trying to switch to deactivating")
        transition = Transition()
        transition.id = Transition.TRANSITION_DEACTIVATE
        transition.label = "deactivate"
        self.change_state(transition, client)
        self.get_logger().info("Deactivating OK, now inactive")

    def cleanup_(self, client):
        # Inactive to Unconfigured
        self.get_logger().info("Trying to switch to cleaning up")
        transition = Transition()  
        transition.id = Transition.TRANSITION_CLEANUP
        transition.label = "cleanup"
        self.change_state(transition, client)
        self.get_logger().info("Cleaning up OK, now unconfigured")

    def initialize_drone_pose_cb(self, request, response):
        self.get_logger().info("Configuring drone_pose lifecycle node")
        self.configure_(self.drone_pose_client)
        self.ready_to_initialize_sub = self.create_subscription(
            Bool, "/ready_to_activate", self.ready_to_activate_cb, 10, callback_group=MutuallyExclusiveCallbackGroup())
        # Wait for the subscriber callback to finish
        self.subscriber_callback_event.wait()

        # Continue with the activation once the subscriber callback is complete
        if self.drone_pose_is_activated:
            self.activate_(self.drone_pose_client)
            response.success = True
        else:
            response.success = False

        return response

    
    def ready_to_activate_cb(self, msg):
        if self.drone_pose_is_activated == False:
            self.get_logger().info("Ready to Actvate")
            self.activate_(self.drone_pose_client)
            self.drone_pose_is_activated = True
            self.subscriber_callback_event.set()




def main(args=None):
    rclpy.init(args=args)
    node = DroneLifecycleNodeManager()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()
    