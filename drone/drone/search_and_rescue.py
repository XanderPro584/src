#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

from yolov8_msgs.msg import Yolov8Inference, InferenceResult
from drone_interfaces.msg import LocalWaypoint
from drone_interfaces.action import GoToWaypoint

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from example_interfaces.msg import Bool
from mavros_msgs.srv import CommandBool

class SearchAndRescueNode(Node): 
    def __init__(self):
        super().__init__("search_and_rescue") 
        self.get_logger().info("Search and Rescue Node has been started")

        self.person_detected = False
        self.waypoint_count = 0
        self.waypoint_list = []
        self.generate_waypoint_list()


        self.yolo_subscriber = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.yolo_callback,
            10)

        #Client to cancel all goals, reference drone_pose.py
        self.cancel_all_goals_client = self.create_client(
            CommandBool, "/cancel_all_goals")
        
        self.go_to_waypoint_client = ActionClient(
            self, GoToWaypoint, "go_to_waypoint")


        # Publisher for /initialize_drone_pose
        self.initialize_drone_pose_pub = self.create_publisher(
            Bool, "/initialize_drone_pose", 10)
        
        # subscriber to drone_pose_is_activated
        self.drone_pose_is_activated_sub = self.create_subscription(
            Bool, "/drone_pose_is_activated", self.drone_pose_is_activated_callback, 10)
        

        self.initialize_drone_pose_lifecycle_node(True)
    

    def initialize_drone_pose_lifecycle_node(self, msg):
        self.get_logger().info("Initializing drone_pose lifecycle node")
        data = Bool()
        data.data = msg
        self.initialize_drone_pose_pub.publish(data)  

    def drone_pose_is_activated_callback(self, data):
        self.get_logger().info("Drone pose is activated, sending first goal")
        self.send_next_goal()

    def generate_waypoint_list(self):
        self.get_logger().info("Generating waypoint list")
        for i in range(20):
            self.waypoint_list.append([0.0, float(0.0 + i*20), 15.0, 0.0])
            self.waypoint_list.append([0.0, float(20.0 + i*20), 15.0, 0.0])
            self.waypoint_list.append([20.0, float(20.0 + i*20), 15.0, 90.0])
            self.waypoint_list.append([20.0, float(20.0 + i*20), 15.0, 90.0])
        self.get_logger().info("Waypoint list generated")

                
    def yolo_callback(self, data: Yolov8Inference):
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
                area = width * height

                self.person_detected = True

        if self.person_detected:
            self.get_logger().info("Person detected")
            self.cancel_all_goals()
        
    def send_goal(self, target_waypoint):
        self.go_to_waypoint_client.wait_for_server()
        goal_msg = GoToWaypoint.Goal()
        goal_msg.goal_position = target_waypoint
        # log the goal position that it is sending
        self.get_logger().info("Sending goal request to: " + str(goal_msg.goal_position))
        self.go_to_waypoint_client.\
            send_goal_async(goal_msg, feedback_callback=self.feedback_callback).\
                add_done_callback(self.goal_response_callback)
        
    def feedback_callback(self, feedback_msg):
        self.get_logger().info("Current Position: " + str(feedback_msg.feedback.current_position))

    def goal_response_callback(self, future):
        self.goal_handle: ClientGoalHandle = future.result()
        if self.goal_handle.accepted:
            self.get_logger().info("Goal accepted")
            self.goal_handle.get_result_async().\
            add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().info("Goal rejected")
            return
        
    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal accepted")
            # start next goal
            self.send_next_goal()

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("Goal aborted.")

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Goal canceled.")

        self.get_logger().info("Final position: " + str(result.final_position)) 

    def cancel_goal(self):
        self.get_logger().info("Canceling goal")
        self.goal_handle.cancel_goal_async()
        
    def cancel_all_goals(self):
        self.get_logger().info("Canceling all goals")
        self.cancel_all_goals_client.wait_for_service()
        request = CommandBool.Request()
        request.value = True
        future = self.cancel_all_goals_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.success:
            self.get_logger().info("All goals canceled")
        else:
            self.get_logger().info("Failed to cancel all goals")

    def send_next_goal(self):
        waypoint = LocalWaypoint()
        waypoint.x = self.waypoint_list[self.waypoint_count][0]
        waypoint.y = self.waypoint_list[self.waypoint_count][1]
        waypoint.z = self.waypoint_list[self.waypoint_count][2]
        waypoint.psi = self.waypoint_list[self.waypoint_count][3]
        self.send_goal(waypoint)

        self.waypoint_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = SearchAndRescueNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()