#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial

from mavros_msgs.srv import CommandBool, CommandInt, SetMode, WaypointClear, WaypointPush
from mavros_msgs.msg import Waypoint, State

# self.mode_client = self.create_client(SetMode, 'mavros/set_mode')
# self.mission_clear_client = self.create_client(WaypointClear, 'mavros/mission/clear')
# self.push_waypoint_client = self.create_client(WaypointPush, 'mavros/mission/push')
# self.state_sub = self.create_subscription(State, 'mavros/state', self.state_callback, 10)



class StartMissionNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("start_mission") # MODIFY NAME

        self.is_armed = False 
        self.arm_attempts = 0
        self.is_guided_mCommandIntode = False
        self.call_arm_server() #ATTEMPT TO ARM

    def push_waypoint(self, x_lat, y_long, z_alt):
        client = self.create_client(WaypointPush, "mavros/mission/push")
        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for Server Waypoint Push...")

        wp = Waypoint()
        wp.frame = 3
        wp.command = 16
        wp.is_current = True
        wp.autocontinue = True
        wp.x_lat = float(x_lat)
        wp.y_long = float(y_long)
        wp.z_alt = float(z_alt)
        wp.param1 = float(0)
        wp.param2 = float(0)
        wp.param3 = float(0)
        wp.param4 = float(0)

        wp_list = WaypointPush.Request()
        wp_list.waypoints = [wp]
        
        future = client.call_async(wp_list)
        future.add_done_callback(partial(self.callback_call_push_waypoint, x_lat=x_lat, y_long=y_long, z_alt=z_alt))

    def callback_call_push_waypoint(self, future, x_lat, y_long, z_alt):
        try: 
            response = future.result()
            self.get_logger().info(f"Pushed wayoint: {response.success}, Coords: {x_lat}, {y_long}, {z_alt}")

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    #WAIT UNTIL ARMED TO PROCEDE TO GUIDED
    def check_arm_status(self):

        if self.is_armed:
            self.arm_timer.cancel()
            self.call_set_mode_server("GUIDED") #SET MODE TO GUIDED

        self.arm_attempts += 1
        if self.arm_attempts >= 10:
            self.get_logger().error("Arm timeout. Check Console")

    #ARM THE DRONE
    def call_arm_server(self):
        self.arm_timer = self.create_timer(1.0, self.check_arm_status) #CONSTANTLY CHECK IF IT IS ARMED

        client = self.create_client(CommandBool, "mavros/cmd/arming")
        while not client.wait_for_service(1):
            self.get_logger().warn("waiting for Server Arming...")

        request = CommandBool.Request()
        request.value = True

        future = client.call_async(request)
        future.add_done_callback(self.callback_call_arm_)

    def callback_call_arm_(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Arm was {response.success}")
            self.is_armed = True

        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")

    #CHANGE THE MODE AND THEN PUSH A WAYPOINT
    def check_mode(self):
        if self.is_guided_mode:
            self.is_guided_timer.cancel()
            self.push_waypoint(15.5,16.3,75.2)
        else: 
            self.get_logger().warn("Waiting for guided mode...")

    def call_set_mode_server(self, mode):
        self.is_guided_timer = self.create_timer(1.0, self.check_mode)

        client = self.create_client(SetMode, "mavros/set_mode")
        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for server Set Mode...")

        request = SetMode.Request()
        request.custom_mode = mode

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_set_mode, mode=mode))

    def callback_call_set_mode(self, future, mode):
        try: 
            response = future.result()
            self.get_logger().info(f"Mode Sent = {response.mode_sent}, Mode: {mode}")
            self.is_guided_mode = True

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
        


def main(args=None):
    rclpy.init(args=args)
    node = StartMissionNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()