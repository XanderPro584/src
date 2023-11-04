#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool


class NumberCounterNode(Node): 


    def __init__(self):
        super().__init__("number_counter") 

        self.counter_ = Int64()
        self.counter_.data = 0
        
        self.publisher_ = self.create_publisher(Int64, "number_count", 10)
        self.subscriber_ = self.create_subscription(Int64, "number", self.publish_counter, 10)
        self.get_logger().info("Number Counter has started")
        self.server_ = self.create_service(SetBool, "reset_number", self.callback_reset_number)

    def publish_counter(self, msg):
        self.counter_.data += msg.data
        self.get_logger().info(f"I heard {msg.data}, the new count is {self.counter_.data}")
        self.publisher_.publish(self.counter_)

    def callback_reset_number(self, request, response):
        if request.data == True:
            self.counter_.data = 0
            response.success = True
        else: 
            response.success = False
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()