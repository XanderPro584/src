#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.srv import TurnOnLed
import random as rand


class BatteryNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("battery") # MODIFY NAME

        self.declare_parameter("led_number", 1)

        self.state = False
        self.set_led_timer_ = self.create_timer(1, self.create_battery_status,)

    def create_battery_status(self):
        self.state = not self.state
        self.call_set_panel_state_server(self.get_parameter("led_number").value, self.state)


    def call_set_panel_state_server(self, led_num, state):
        self.set_panel_state_client_ = self.create_client(TurnOnLed, "set_led_panel")
        while not self.set_panel_state_client_.wait_for_service(1):
            self.get_logger().warn("Waiting for Set Panel State Service to start")

        request = TurnOnLed.Request()
        request.led_number = led_num
        request.state = state

        future = self.set_panel_state_client_.call_async(request)
        future.add_done_callback(self.callback_set_panel_state)


    def callback_set_panel_state(self, future):
        try: 
            response = future.result()
            states = {
                True : "Success",
                False : "Failure"
            }

            self.get_logger().info(f"Calling Set Led Panel State Server was a {states[response.success]}!")
        except Exception as e:
            self.get_logger().error(f"it didn't work: {e}")


        
        


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()