#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from drone_interfaces.action import GoToWaypoint
from drone_interfaces.msg import LocalWaypoint

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus


class WaypointSenderNode(Node): 
    def __init__(self):
        super().__init__("wayoint_sender") 
        self.go_to_waypoint_client = ActionClient(self, GoToWaypoint, "go_to_waypoint")
        self.get_logger().info("Waypoint Sender Node has been started")

        # self.cancel_timer = self.create_timer(5.0, self.cancel_goal)

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

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("Goal aborted.")

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Goal canceled.")

        self.get_logger().info("Final position: " + str(result.final_position)) 

    def cancel_goal(self):
        self.get_logger().info("Canceling goal")
        self.goal_handle.cancel_goal_async()
        #destroy timer
        self.cancel_timer.destroy()



def main(args=None):
    rclpy.init(args=args)
    node = WaypointSenderNode() 
    target_waypoint = LocalWaypoint()
    #make a square
    target_waypoint.x = 300.0
    target_waypoint.y = 0.0
    target_waypoint.z = 5.0
    target_waypoint.psi = -90.0
    node.send_goal(target_waypoint)
    time.sleep(0.1)


    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()