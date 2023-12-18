#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

# import multithreading and reentrantcallbackgroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


from example_interfaces.msg import Bool


class DroneLifecycleNodeManager(Node):
    def __init__(self):
        super().__init__("lifecycle_manager")
        self.get_logger().info("Drone Lifecycle Manager has been started")
        self.drone_pose_client = self.create_client(
            ChangeState, "/drone_pose/change_state", )

        # Subscriber for starting drone_pose node
        self.initialize_drone_pose_lifecycle_node_sub = self.create_subscription(
            Bool, "/initialize_drone_pose", self.initialize_drone_pose_lifecycle_node, qos_profile=10, )

        # self.initialize_drone_pose_lifecycle_node(True)

    def change_state(self, transition: Transition, client):
        client.wait_for_service()
        request = ChangeState.Request()
        request.transition = transition
        future = client.call_async(request)
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

    def initialize_drone_pose_lifecycle_node(self, msg):
        self.get_logger().info("Configuring drone_pose lifecycle node")
        self.drone_pose_is_activated = False
        self.configure_(self.drone_pose_client)
        self.ready_to_initialize_sub = self.create_subscription(
            Bool, "/ready_to_activate", self.ready_to_activate_cb, qos_profile=10, )
        self.get_logger().info("created ready to initialize subscriber")


    def ready_to_activate_cb(self, msg):
        if self.drone_pose_is_activated == False:
            self.get_logger().info("Ready to Actvate")
            self.activate_(self.drone_pose_client)
            self.drone_pose_is_activated = True
            # Publisher that says when the drone_pose node is activated
            self.drone_pose_is_activated_pub = self.create_publisher(
                Bool, "/drone_pose_is_activated", qos_profile=10)

            self.drone_pose_is_activated_pub.publish(Bool(data=True))

def main(args=None):
    rclpy.init(args=args)
    node = DroneLifecycleNodeManager()
    rclpy.spin(node, )
    rclpy.shutdown()
    