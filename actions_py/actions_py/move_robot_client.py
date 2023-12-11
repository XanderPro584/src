#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import time

from my_robot_interfaces.action import MoveRobot
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus, ClientGoalHandle
from threading import Thread
import sys
# from rclpy.executors import MultiThreadedExecutor


class MoveRobotClientNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("move_robot_client") # MODIFY NAME

        #Start action client 
        self.move_robot_client = ActionClient(self, MoveRobot, "/move_robot")

        self.get_logger().info("Move Robot Client node started")

    def send_goal(self, position, velocity):
        self.move_robot_client.wait_for_server()

        goal = MoveRobot.Goal()
        goal.position = position
        goal.velocity = velocity

        self.get_logger().info(f"Sending goal request: {goal}")
        self.move_robot_client. \
            send_goal_async(goal, feedback_callback=self.goal_feedback_callback). \
                add_done_callback(self.goal_response_callback)
        
        #Timer to cancel the goal after 3 seconds
        # self.timer = self.create_timer(3, self.cancel_goal)
        
    def goal_feedback_callback(self, feedback_msg):
        current_position = feedback_msg.feedback.current_position
        self.get_logger().info(f"Current position: {current_position}")

    def goal_response_callback(self, future):
        self.goal_handle: ClientGoalHandle = future.result()
        if self.goal_handle.accepted:
            self.get_logger().info("Goal accepted")
            self.goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

        else:
            self.get_logger().warn("goal rejected")

    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Goal Succeded")

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("Goal Aborted")

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Goal Canceled")

        self.get_logger().info(f"Final Position: {result.final_position}")

    def cancel_goal(self):
        self.get_logger().info("Sending a cancel request")
        self.goal_handle.cancel_goal_async()
        self.timer.cancel()



def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotClientNode() # MODIFY NAME
    spin_thread = Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()

    node.send_goal(60, 2)
    time.sleep(2)
    node.send_goal(30, 3)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()