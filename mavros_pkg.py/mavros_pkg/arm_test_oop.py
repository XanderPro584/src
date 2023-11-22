#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial

from mavros_msgs.srv import CommandBool


class ArmTestOOPNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("arm_test_oop") # MODIFY NAME
        self.call_arm_server(True)

    def call_arm_server(self, value):
        client = self.create_client(CommandBool, "mavros/cmd/arming")
        while not client.wait_for_service(1):
            self.get_logger().warn("waiting for Server Arming")

        request = CommandBool.Request()
        request.value = value

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_arm_, value=value))

    def callback_call_arm_(self, future, value):
        try:
            response = future.result()
            self.get_logger().info(f"Arm was {response.success}, when called value: {value}")

        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArmTestOOPNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()