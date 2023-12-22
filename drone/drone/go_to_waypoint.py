#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from example_interfaces.msg import Float64
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from drone_interfaces.msg import LocalWaypoint, LocalFrameOffsets
from drone_interfaces.action import GoToWaypoint
from mavros_msgs.srv import CommandBool

import threading
import time
import numpy as np
from math import pow, sqrt, radians, sin, cos


class GoToWaypointNode(Node): 
    def __init__(self):
        super().__init__("go_to_waypoint") 
        self.get_logger().info("Go To Waypoint Node has been started")

        self.local_pose = Point()
        self.local_heading = Float64()
        self.local_frame_offsets = LocalFrameOffsets()
        self.current_pose_g = PoseStamped()
        self.waypoint_g = PoseStamped()
        self.local_desired_heading_g = 0.0

        self.rotation_theta = 0.0
        self.translation_x = 0.0
        self.translation_y = 0.0

        self.goal_queue = []
        self.goal_lock = threading.Lock()
        self.goal_handle: ServerGoalHandle = None
        self.abort_current_goal = False



        # Volatile Quality of Service profile
        self.volatile_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.local_position_sub = self.create_subscription(
            Point, "/local_position", self.pose_callback, self.volatile_qos_profile)
        self.local_heading_sub = self.create_subscription(
            Float64, "/local_heading", self.heading_callback, self.volatile_qos_profile)
        self.currentPos = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.pose_cb, qos_profile=self.volatile_qos_profile,)
        self.local_frame_offsets_sub = self.create_subscription(
            LocalFrameOffsets, "/local_frame_offsets", self.local_frame_offsets_callback, qos_profile=self.volatile_qos_profile,)
        self.local_pos_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", qos_profile=self.volatile_qos_profile)
        self.cancel_all_goals_srv = self.create_service(
            CommandBool, "/cancel_all_goals", self.cancel_all_goals_cb, callback_group=MutuallyExclusiveCallbackGroup())

        self.go_to_waypoint_server = ActionServer(
            self,
            GoToWaypoint,
            "go_to_waypoint",
            goal_callback=self.goal_callback,
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=ReentrantCallbackGroup()
        )

    # Action server callbacks
    def goal_callback(self, goal_request: GoToWaypoint.Goal):
        # Use the check_waypoint_reached function to see if the goal waypoint is within 500 meters of the current position
        if self.check_waypoint_reached(pos_tol=500, angle_tol=10):
            # Accept the goal
            self.get_logger().info("Goal within max distance: ACCEPTED")
            return GoalResponse.ACCEPT
        else:
            # Reject the goal
            self.get_logger().info("Goal not within max distance or server is not activated: REJECTED")
            return GoalResponse.REJECT
        
    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        # Add the goal to the queue
        with self.goal_lock:
            if self.goal_handle is not None:
                self.goal_queue.append(goal_handle)
                self.get_logger().info("Goal added to queue")
            else:
                goal_handle.execute()

    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Executing goal...")
        with self.goal_lock:
            self.goal_handle = goal_handle

        goal_position = goal_handle.request.goal_position
        self.get_logger().info("Going to {}".format(goal_position))
        self.set_destination(
            goal_position.x, 
            goal_position.y, 
            goal_position.z, 
            goal_position.psi)
        
        current_position = LocalWaypoint()
        result = GoToWaypoint.Result()
        feedback = GoToWaypoint.Feedback()

        while self.check_waypoint_reached() == 0:
            # Publish the current position
            current_position.x = self.local_pose.x
            current_position.y = self.local_pose.y
            current_position.z = self.local_pose.z
            current_position.psi = self.local_heading.data

            feedback.current_position = current_position
            result.final_position = current_position

            # Reasons to kill the goal
            if goal_handle.is_cancel_requested:
                self.get_logger().warn("Goal canceled")
                goal_handle.canceled()
                self.send_current_position_as_waypoint()
                self.process_next_goal_in_queue()
                return result

            elif not goal_handle.is_active:
                self.get_logger().warn("Goal no longer active. Aborting...")
                goal_handle.abort()
                self.send_current_position_as_waypoint()
                self.process_next_goal_in_queue()
                return result
            
            elif self.abort_current_goal:
                self.get_logger().warn("Aborting current goal")
                goal_handle.abort()
                self.abort_current_goal = False
                self.send_current_position_as_waypoint()
                self.process_next_goal_in_queue()
                return result
            goal_handle.publish_feedback(feedback)
            time.sleep(0.5)

        else: 
            # Send the result of the action
            goal_handle.succeed()
            self.get_logger().info("Goal reached")
            result.final_position = current_position
            self.process_next_goal_in_queue()
            return result
        
    def send_current_position_as_waypoint(self):
        self.get_logger().info("Sending coordinate of current position as waypoint")
        self.set_destination(
            self.local_pose.x,
            self.local_pose.y,
            self.local_pose.z,
            self.local_heading.data
        )
        time.sleep(1)
        
    def cancel_all_goals_cb(self, request, response):
        self.get_logger().warn("Cancel all goals request received")
        self.abort_current_goal = True
        self.goal_queue = []
        self.goal_handle = None
        return response

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received Cancel Request for the last accepted goal")
        return CancelResponse.ACCEPT
    
    def process_next_goal_in_queue(self):
        with self.goal_lock:
            if len(self.goal_queue) > 0:
                self.goal_queue.pop(0).execute()
            else:
                self.goal_handle = None

    # Waypoint Setting Functions
    def set_heading(self, heading):
        heading = heading + self.rotation_theta
        # self.get_logger().info("The desired heading is {}".format(self.local_desired_heading_g))
        yaw = radians(heading)
        pitch = 0.0
        roll = 0.0

        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)

        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)

        qw = cy * cr * cp + sy * sr * sp
        qx = cy * sr * cp - sy * cr * sp
        qy = cy * cr * sp + sy * sr * cp
        qz = sy * cr * cp - cy * sr * sp

        orientation = Quaternion()
        orientation.x = qx
        orientation.y = qy
        orientation.z = qz
        orientation.w = qw

        return orientation

    def set_destination(self, x, y, z, psi):
        orientation = self.set_heading(psi)

        original_point = self.transform_point(x, y)

        position = Point()
        position.x = original_point[0]
        position.y = original_point[1]
        position.z = z

        self.get_logger().info("Destination set to x:{} y:{} z:{} origin frame".format(position.x, position.y, position.z))

        self.waypoint_g.pose.orientation = orientation
        self.waypoint_g.pose.position = position
        self.local_pos_pub.publish(self.waypoint_g)
    
    def transform_point(self, x, y):
        # Apply rotation first and then translation
        theta = radians(self.rotation_theta - 90)
        rotation_matrix = np.array([[cos(theta), -sin(theta)],
                                    [sin(theta), cos(theta)]])
        rotated_point = np.dot(rotation_matrix, np.array([x, y]))

        translated_point = rotated_point + np.array([self.translation_x, self.translation_y])

        return translated_point
    
    def inverse_transform_point(self, x, y):
        # Apply inverse translation first and then inverse rotation
        inverse_translated_point = np.array([x, y]) - np.array([self.translation_x, self.translation_y])

        theta = radians(-(self.rotation_theta - 90))
        inverse_rotation_matrix = np.array([[cos(theta), -sin(theta)],
                                           [sin(theta), cos(theta)]])
        inverse_rotated_point = np.dot(inverse_rotation_matrix, inverse_translated_point)

        return inverse_rotated_point
    
    def check_waypoint_reached(self, pos_tol=0.5, angle_tol=0.08):
        dx = abs(self.waypoint_g.pose.position.x - self.current_pose_g.pose.position.x)
        dy = abs(self.waypoint_g.pose.position.y - self.current_pose_g.pose.position.y)
        dz = abs(self.waypoint_g.pose.position.z - self.current_pose_g.pose.position.z)

        dMag = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))

        # Convert quaternion to euler angles for both current and desired orientations
        current_euler = self.quaternion_to_euler(self.current_pose_g.pose.orientation)
        desired_euler = self.quaternion_to_euler(self.waypoint_g.pose.orientation)

        # Extract yaw angles from euler angles
        current_yaw = current_euler[2]  # Yaw angle is at index 2
        desired_yaw = desired_euler[2]

        # Calculate angular difference between current and desired yaw angles
        yaw_angle_diff = abs(current_yaw - desired_yaw)
        self.get_logger().info("Yaw angle difference: {}".format(yaw_angle_diff))
        self.get_logger().info("dMag: {}".format(dMag))

        if dMag < pos_tol and yaw_angle_diff < angle_tol:
            return 1
        else:
            return 0    

    def quaternion_to_euler(self, q):
        # Convert quaternion to euler angles
        t0 = +2.0 * (q.w * q.x + q.y * q.z)
        t1 = +1.0 - 2.0 * (q.x**2 + q.y**2)
        roll_x = np.arctan2(t0, t1)

        t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)

        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y**2 + q.z**2)
        yaw_z = np.arctan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians

        


    # Variable updates callbacks
    def pose_callback(self, msg):
        self.local_pose = msg

    def heading_callback(self, msg):
        self.local_heading = msg

    def pose_cb(self, msg):
        self.current_pose_g = msg

    def local_frame_offsets_callback(self, msg):
        self.local_frame_offsets = msg
        self.rotation_theta = msg.rotation_theta
        self.translation_x = msg.translation_x
        self.translation_y = msg.translation_y




def main(args=None):
    rclpy.init(args=args)
    node = GoToWaypointNode() 
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()