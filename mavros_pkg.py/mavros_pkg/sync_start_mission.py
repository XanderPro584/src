#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import sys
from threading import Thread

from example_interfaces.srv import AddTwoInts

from mavros_msgs.srv import CommandBool, CommandInt, SetMode, WaypointClear, WaypointPush, CommandTOL
from mavros_msgs.msg import Waypoint, State


class SyncStartMissionNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("sync_start_mission") # MODIFY NAME
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.clear_waypoint_client = self.create_client(WaypointClear, 'mavros/mission/clear')
        self.push_waypoint_client = self.create_client(WaypointPush, 'mavros/mission/push')
        self.command_client = self.create_client(CommandInt, 'mavros/cmd/command_int')
        self.takeoff_client = self.create_client(CommandTOL, 'mavros/cmd/takeoff')

        self.wp_list = []
    
    def ready_client(self, client, server, request):
        while not client.wait_for_service(1):
            self.get_logger().warn(f"Waiting for {server} server")
        result = client.call(request)
        return result
    
    def takeoff_request(self, altitude=10):
        request = CommandTOL.Request()
        request.altitude = float(altitude)
        result = self.ready_client(self.takeoff_client, "Takeoff", request)
        self.get_logger().info(f"Takeoff response = {result.success}")
        return result.success

    
    def command_request(self, command):
        request = CommandInt.Request()
        request.command = command
        result = self.ready_client(self.command_client, "Command", request)
        self.get_logger().info(f"Command {command} response = {result.success}")
        return result.success
    
    def new_waypoint(self, frame=3, command=16, x=0.0, y=0.0, z=0.0, is_current=False):
        wp = Waypoint()
        
        wp.frame = frame
        wp.command = command
        wp.is_current = is_current
        wp.autocontinue = True
        wp.x_lat = float(x)
        wp.y_long = float(y)
        wp.z_alt = float(z)
        wp.param1 = float(0)
        wp.param2 = float(0)
        wp.param3 = float(0)
        wp.param4 = float(0)

        self.wp_list.append(wp)

    def push_waypoint_request(self):
        request = WaypointPush.Request()
        request.waypoints = self.wp_list
        request.start_index = 0

        result = self.ready_client(self.push_waypoint_client, "Push Waypoint", request)
        self.get_logger().info(f"Push Waypoint response = {result.success}")

        self.wp_list = []
        return result.success
    
    def clear_waypoint_request(self):
        request = WaypointClear.Request()
        result = self.ready_client(self.clear_waypoint_client, "Clear Waypoint", request)
        self.get_logger().info(f"Clear Waypoint response = {result.success}")
        return result.success

    def arm_request(self):
        request = CommandBool.Request()
        request.value = True
        result = self.ready_client(self.arm_client, "Arm", request)
        self.get_logger().info(f"Arm response = {result.success}")
        return result.success

    
    def set_mode_request(self, mode):
        request = SetMode.Request()
        request.custom_mode = mode
        result = self.ready_client(self.set_mode_client, "Set Mode", request)
        self.get_logger().info(f"Set Mode response = {result.mode_sent}")
        return result.mode_sent
        


def main(args=None):
    rclpy.init(args=args)
    node = SyncStartMissionNode() # MODIFY NAME
    spin_thread = Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()

    node.clear_waypoint_request()
    node.new_waypoint(command=22, is_current=True, x=-35.3632622, y=149.1652375, z=10)
    node.new_waypoint(x=-35.365, y=149.165, z=29)
    node.new_waypoint(x=-35.360, y=149.170, z=16.7)
    node.new_waypoint(command=21, x=-35.3632622, y=149.1652375, z=1)
    node.push_waypoint_request()
    node.set_mode_request("GUIDED")
    time.sleep(3)
    node.arm_request()
    time.sleep(3)
    node.takeoff_request()
    time.sleep(20)
    node.set_mode_request("AUTO")
    time.sleep(1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()