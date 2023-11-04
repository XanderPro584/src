#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import LedPanelState
from my_robot_interfaces.srv import TurnOnLed


class LedPanelNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("led_panel") # MODIFY NAME

        self.led_states = [0,0,0]
        self.set_led_ = self.create_service(TurnOnLed, "set_led_panel", self.callack_set_led_panel)
        self.led_state_publisher_ = self.create_publisher(LedPanelState, "panel_state", 10)
        self.led_state_publish_timer_ = self.create_timer(1, self.publish_led_state)

        self.get_logger().info("Led Panel has started")

    def publish_led_state(self):
        msg = LedPanelState()
        msg.led_1 = self.led_states[0]
        msg.led_2 = self.led_states[1]
        msg.led_3 = self.led_states[2]

        self.led_state_publisher_.publish(msg)

    def callack_set_led_panel(self, request, response):
        if request.state:
            self.led_states[request.led_number - 1] = 1
        else:
            self.led_states[request.led_number -1] = 0

        self.get_logger().info(str(self.led_states))

        response.success = True
        return response



def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()