#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from example_interfaces.msg import Float64
from yolov8_msgs.msg import Yolov8Inference

from drone_interfaces.action import ReadyDrone
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from drone_interfaces.srv import CommandVelocity


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
        self.current_heading = Float64()
        self.person_detected = False
        self.drone_is_ready = False

        self.bounding_box = []
        self.bounding_box_center = []
        self.bounding_box_area = 0.0

        self.local_pose_sub = self.create_subscription(
            Point, "/local_position", self.local_pose_callback, self.volatile_qos_profile, callback_group=ReentrantCallbackGroup())
        self.local_heading_sub = self.create_subscription(
            Float64, "/local_heading", self.local_heading_callback, self.volatile_qos_profile, callback_group=ReentrantCallbackGroup())
        
        self.ready_drone_client = ActionClient(
            self, ReadyDrone, "ready_drone", callback_group=ReentrantCallbackGroup())
        self.send_cmd_vel_client = self.create_client(
            CommandVelocity, "/set_cmd_vel", callback_group=ReentrantCallbackGroup())
        
        self.ready_drone()
        
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
            time.sleep(10)
            self.drone_is_ready = True
            self.drone_is_ready_cb()

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("Goal aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Canceled")

    def drone_is_ready_cb(self):
        if self.move_foreward_until_person():
                self.yolo_inference_sub = self.create_subscription(
                    Yolov8Inference, "/Yolov8_Inference", self.yolo_inference_callback, 10, callback_group=ReentrantCallbackGroup())

    # Movement functions
    def move_foreward_until_person(self):
        self.get_logger().info("Moving foreward until person is detected")
        cmd_vel = CommandVelocity.Request()
        cmd_vel.linear_y = 2.0
        cmd_vel.linear_z = 1.0
        self.send_cmd_vel_client.wait_for_service()
        response = self.send_cmd_vel_client.call(cmd_vel)
        return response
    
    def stop(self):
        self.get_logger().info("Stopping")
        cmd_vel = CommandVelocity.Request()
        self.send_cmd_vel_client.wait_for_service()
        response = self.send_cmd_vel_client.call(cmd_vel)
        return response
    
    # Detection functions
    def yolo_inference_callback(self, data):
        # Check if there is a person detected
        for r in data.yolov8_inference:
            class_name = r.class_name
            if class_name == "person":
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
                self.bounding_box_center = [x_center, y_center]
                if not self.person_detected:
                    self.person_detected = True

                    self.person_detected_cb()

    def person_detected_cb(self):
        self.stop()
        time.sleep(1)

        person_is_in_frame = False
        person_is_at_proper_distance = False
        while not person_is_in_frame:
            cmd_vel = CommandVelocity.Request()
            person_is_in_frame, linear_x, linear_y = self.move_within_frame()
            cmd_vel.linear_x = linear_x
            cmd_vel.linear_y = linear_y
            self.send_cmd_vel_client.wait_for_service()
            self.send_cmd_vel_client.call(cmd_vel)
            time.sleep(0.1)

        else:
            self.get_logger().info("Person is in the center of the frame")
            self.stop()
            time.sleep(1)
            while not person_is_at_proper_distance or not person_is_in_frame:
                cmd_vel = CommandVelocity.Request()
                person_is_in_frame, linear_x, linear_y = self.move_within_frame()
                person_is_at_proper_distance, linear_z = self.get_proper_distance_from_person()
                cmd_vel.linear_x = linear_x
                cmd_vel.linear_y = linear_y
                if not person_is_at_proper_distance:
                    linear_z = linear_z / 3
                cmd_vel.linear_z = linear_z
                self.send_cmd_vel_client.wait_for_service()
                self.send_cmd_vel_client.call(cmd_vel)
                time.sleep(0.1)

        if person_is_at_proper_distance and person_is_in_frame:
            self.get_logger().info("Person is at the proper distance")
            self.stop()
            time.sleep(1)
            self.get_logger().info("Orbiting person")
            self.orbit_person(left=True, velocity=0.8)

    def orbit_person(self, left=True, velocity=0.8):
        if left:
            # starting_angular = 3
            linear_x = -velocity
        else:
            # starting_angular = -3
            linear_x = velocity

        start_heading = self.current_heading.data
        start_time = time.time()
        # Orbit until the start heading is the same as current heading AND it has been at least 10 seconds
        while (abs(start_heading - self.current_heading.data)) > 8 or (time.time() - start_time) < 30:
            self.get_logger().info(f"Current heading: {self.current_heading.data}")
            self.get_logger().info(f"Start heading: {start_heading}")
            self.get_logger().info(f"Time: {time.time() - start_time}")
            self.adjust_orbit(linear_x)
            time.sleep(0.1)

        else:
            self.get_logger().info("Done Orbiting")
            self.stop()
            time.sleep(1)



    def adjust_orbit(self, linear_x):
        cmd_vel = CommandVelocity.Request()
        cmd_vel.linear_x = linear_x
        cmd_vel.angular_z = self.adjust_angular_velocity()
        a, b, linear_y = self.move_within_frame()
        cmd_vel.linear_y = linear_y * 2
        self.send_cmd_vel_client.wait_for_service()
        self.send_cmd_vel_client.call(cmd_vel)
        
    def adjust_angular_velocity(self):
        too_far_left, too_far_right, too_far_up, too_far_down = self.check_person_frame_position()
        if not too_far_left or not too_far_right:
            self.get_logger().info("Person is in the center of the frame")
            angular_z = (320 - self.bounding_box_center[0]) / 8
        elif too_far_right:
            self.get_logger().info("Person is too far right")
            angular_z = -30
        elif too_far_left:
            self.get_logger().info("Person is too far left")
            angular_z = 30
        else:
            angular_z = 0.0

        return angular_z
        # if self.bounding_box_center[0] > 320:
        #     self.get_logger().info("Person is too far right")
        #     # Measure distance from center of box to right edge of frame
        #     distance_to_right_edge = 640 - self.bounding_box_center[0]
        #     # Calculate angular velocity
        #     angular_z = 5*(320 / distance_to_right_edge)
        #     if angular_z > 50:
        #         angular_z = 50
        #     angular_z = -angular_z
        #     self.get_logger().info(f"Angular z: {angular_z}")
        # else:
        #     self.get_logger().info("Person is too far left")
        #     # Measure distance from center of box to left edge of frame
        #     distance_to_left_edge = self.bounding_box_center[0]
        #     # Calculate angular velocity
        #     angular_z = 5*(320 / distance_to_left_edge)
        #     if angular_z > 50:
        #         angular_z = 50
        #     self.get_logger().info(f"Angular z: {angular_z}")
        # return angular_z

        

    def move_within_frame(self):
        # Check if the person is close enough to the center of the frame
        too_far_left, too_far_right, too_far_up, too_far_down = self.check_person_frame_position()
        linear_x = 0.0
        linear_y = 0.0
        if too_far_left:
            self.get_logger().info("Person is too far left")
            linear_x = -0.5
        if too_far_right:
            self.get_logger().info("Person is too far right")
            linear_x = 0.5
        if too_far_up:
            self.get_logger().info("Person is too far up")
            linear_y = 0.5
        if too_far_down:
            self.get_logger().info("Person is too far down")
            linear_y = -0.5
        if not too_far_left and not too_far_right and not too_far_up and not too_far_down:
            self.get_logger().info("Person is in the center of the frame")
            return True, linear_x, linear_y
        else:
            return False, linear_x, linear_y
        

    def get_proper_distance_from_person(self):
        # Check if the person is close enough to the drone
        self.get_logger().info(f"Bounding box area: {self.bounding_box_area}")
        too_close = self.bounding_box_area > 12000
        too_far = self.bounding_box_area < 6000
        linear_z = 0.0
        if too_close:
            self.get_logger().info("Person is too close")
            linear_z = 0.5
        if too_far:
            self.get_logger().info("Person is too far")
            linear_z = -0.5
        if not too_close and not too_far:
            self.get_logger().info("Person is at the proper distance")
            return True, linear_z
        else:
            return False, linear_z


    
    def check_person_frame_position(self):
        # Check if the person is close enough to the center of the frame
        too_far_left = self.bounding_box_center[0] < 260
        too_far_right = self.bounding_box_center[0] > 380
        too_far_up = self.bounding_box_center[1] < 180
        too_far_down = self.bounding_box_center[1] > 300
        return too_far_left, too_far_right, too_far_up, too_far_down



    def local_pose_callback(self, msg):
        self.current_pose = msg
    
    def local_heading_callback(self, msg):
        self.current_heading = msg


def main(args=None):
    rclpy.init(args=args)
    node = OrbitNode() 
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()