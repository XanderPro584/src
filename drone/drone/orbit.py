#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from yolov8_msgs.msg import Yolov8Inference

from drone_interfaces.action import ReadyDrone
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class OrbitNode(Node): 
    def __init__(self):
        super().__init__("orbit") 
        self.get_logger().info("Orbit Node has been started")

        # Volatile Quality of Service profile
        self.volatile_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.current_pose = Point()
        self.current_heading = Float32()
        self.person_detected = False
        self.drone_is_ready = False

        self.bounding_box = []
        self.bounding_box_center = []
        self.bounding_box_area = 0.0

        self.local_pose_sub = self.create_subscription(
            Point, "/local_position", self.local_pose_callback, self.volatile_qos_profile)
        self.local_heading_sub = self.create_subscription(
            Float32, "/local_heading", self.local_heading_callback, self.volatile_qos_profile)
        self.yolo_inference_sub = self.create_subscription(
            Yolov8Inference, "/Yolov8_Inference", self.yolo_inference_callback, 10, callback_group=ReentrantCallbackGroup())
        self.cmd_vel_pub = self.create_publisher(
            Twist, "mavros/setpoint_velocity/cmd_vel_unstamped", 10, callback_group=MutuallyExclusiveCallbackGroup())
        
        self.ready_drone_client = ActionClient(
            self, ReadyDrone, "ready_drone", callback_group=ReentrantCallbackGroup())
        
        self.ready_drone()
        # self.move_foreward()
        
    def ready_drone(self, takeoff_alt=3.0):
        while not self.ready_drone_client.wait_for_server(1):
            self.get_logger().warn("Waiting for ready drone action server...")
        goal = ReadyDrone.Goal()
        goal.altitude = takeoff_alt
        self.get_logger().info(f"Sending goal request: {goal}")
        self.ready_drone_client.send_goal_async(goal).add_done_callback(self.ready_drone_callback)

    def ready_drone_callback(self, future):
        goal_handle: ClientGoalHandle = future.result()
        if goal_handle.accepted:
            self.get_logger().info("Goal accepted")
            goal_handle.get_result_async().add_done_callback(self.ready_drone_result_callback)
        else:
            self.get_logger().warn("Goal rejected")

    def ready_drone_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded")
            self.get_logger().info("Drone is ready to orbit")
            self.drone_is_ready = True

            self.move_foreward()

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("Goal aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Canceled")
    
    def move_foreward(self):
        self.get_logger().info("Moving foreward")
        cmd_vel_msg = Twist()
        for i in range(200):
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.linear.y = 1.0
            self.cmd_vel_pub.publish(cmd_vel_msg)
            time.sleep(0.5)
        
    def local_pose_callback(self, msg):
        self.current_pose = msg
    
    def local_heading_callback(self, msg):
        self.current_heading = msg
    
    def yolo_inference_callback(self, data):
        # Check if there is a person detected
        for r in data.yolov8_inference:
            class_name = r.class_name
            if class_name == "person":
                self.person_detected = True
                # Get the bounding box of the person
                top = r.top
                left = r.left
                bottom = r.bottom
                right = r.right
                # Get the center of the bounding box
                x_center = (top + bottom) / 2
                y_center = (left + right) / 2
                # Get the width and height of the bounding box
                width = bottom - top
                height = right - left
                # Get the area of the bounding box
                self.bounding_box_area = width * height
                self.bounding_box = [top, left, bottom, right]


def main(args=None):
    rclpy.init(args=args)
    node = OrbitNode() 
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()