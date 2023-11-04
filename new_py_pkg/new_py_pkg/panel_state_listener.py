#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import LedPanelState


class PanelStateListenerNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("panel_state_listener") # MODIFY NAME

        self.panel_state_subscriber_ = self.create_subscription(LedPanelState, "panel_state", self.log_panel_state, 10)

    def log_panel_state(self, msg):
        panel_state_dict = {
            1 : 'ON',
            0 : "OFF"
        }

        self.get_logger().info(f"LED 1 {panel_state_dict[msg.led_1]} | LED 2 {panel_state_dict[msg.led_2]} | LED 3 {panel_state_dict[msg.led_3]}")


def main(args=None):
    rclpy.init(args=args)
    node = PanelStateListenerNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()