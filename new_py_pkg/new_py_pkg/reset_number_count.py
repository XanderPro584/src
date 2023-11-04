#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# from functools import partial

from example_interfaces.srv import SetBool


class ResetNumberCount(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("reset_number_count") # MODIFY NAME
        self.timer_ = self.create_timer(5, self.call_reset_number_service)

    def call_reset_number_service(self):
        self.client_ = self.create_client(SetBool, "reset_number")

        while not self.client_.wait_for_service(1):
            self.get_logger().warn("Waiting for server to start")

        request = SetBool.Request()
        request.data = True

        future = self.client_.call_async(request)
        future.add_done_callback(self.callback_call_reset_number)

    def callback_call_reset_number(self, future):
        try:
            response = future.result()
            if response.success == True:
                self.get_logger().info("Successfully reset number")

        except Exception as e:
            self.get_logger().error(f"failed to reset number: {e}")
        


def main(args=None):
    rclpy.init(args=args)
    node = ResetNumberCount() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()