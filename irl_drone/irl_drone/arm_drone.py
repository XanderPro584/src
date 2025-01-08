#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import State
from rclpy.qos import QoSProfile, ReliabilityPolicy

class ArmDroneNode(Node):
    def __init__(self):
        super().__init__("arm_drone_node")
        self.get_logger().info("Arm Drone Node has been started.")

        # QoS Profile (non-reliable)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Non-reliable QoS
            depth=1
        )

        # Create a client to call the arming service
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming", qos_profile=qos_profile)

        # State subscription to monitor the current state of the drone
        self.state_sub = self.create_subscription(State, "/mavros/state", self.state_callback, qos_profile)

        self.current_state = None

    def state_callback(self, msg):
        """Callback to update the drone's current state."""
        self.current_state = msg

    def arm_drone(self):
        """Arms the drone by calling the MAVROS arming service."""
        self.get_logger().info("Attempting to arm the drone...")

        # Wait for the arming service to become available
        self.arm_client.wait_for_service()

        self.get_logger().info("Arming service is available.")

        # Create a request to arm the drone
        request = CommandBool.Request()
        request.value = True

        # Call the service
        future = self.arm_client.call_async(request)

        # Wait for the result
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info("Drone successfully armed.")
        else:
            self.get_logger().error("Failed to arm the drone.")

def main(args=None):
    rclpy.init(args=args)
    node = ArmDroneNode()

    # Ensure that the drone is connected and ready
    while node.current_state is None or not node.current_state.connected:
        rclpy.spin_once(node)
        node.get_logger().info("Waiting for connection to FCU...")

    # Call the function to arm the drone
    node.arm_drone()

    node.get_logger().info("Shutting down Arm Drone Node.")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
