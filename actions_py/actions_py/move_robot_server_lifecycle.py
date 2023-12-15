#!/usr/bin/env python3
import rclpy
import time
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import MoveRobot

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class MoveRobotServerNode(LifecycleNode): 
    def __init__(self):
        super().__init__("move_robot_server") 

        self.declare_parameter("node_name", rclpy.Parameter.Type.STRING)

        self.get_logger().info("Move Robot Server node started")

    # Configure move robot server and initialize variables
    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info("Configuring")
        self.min_distance = 0
        self.max_distance = 100
        self.robot_current_position = 50

        self.goal_handle: ServerGoalHandle = None
        self.goal_queue = []

        #Declare action server
        self.move_robot_server = ActionServer(
            self,
            MoveRobot,
            "/move_robot",
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("Activating")
        
        return TransitionCallbackReturn.SUCCESS

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        if self.goal_handle is not None:
            self.goal_queue.append(goal_handle)
        else: 
            goal_handle.execute()

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a cancel request")
        return CancelResponse.ACCEPT 

    def goal_callback(self, goal_request: MoveRobot.Goal):
        #Check that the goal position and velocity is within reason
        self.get_logger().info(f"goal position: {goal_request.position}, goal velocity: {goal_request.velocity}")
        if goal_request.position > self.min_distance \
            and goal_request.position < self.max_distance \
            and goal_request.velocity > 0:
            self.get_logger().info("Goal is within reason. ACCEPTING")
            return GoalResponse.ACCEPT 
        
        else:
            self.get_logger().info("Goal is not within reason. REJECTING")
            return GoalResponse.REJECT 
        

    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.goal_handle = goal_handle
        result = MoveRobot.Result()
        feedback = MoveRobot.Feedback()

        position = goal_handle.request.position
        velocity = goal_handle.request.velocity

        #Check that it is not already at the target
        if position == self.robot_current_position:
            result.final_position = self.robot_current_position
            self.process_next_goal()
            return result
        
        #Otherwise move the robot until it gets to the target position
        while not position == self.robot_current_position:
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Canceling the goal")
                goal_handle.canceled()
                result.final_position = self.robot_current_position
                self.process_next_goal()
                return result

            self.get_logger().info(f"current position: {self.robot_current_position}")
            #Check if it is closer than the step size. If yes, then it is completed
            if abs(position - self.robot_current_position) < velocity:
                self.robot_current_position = position

            #Check if it needs to go backwards
            elif position < self.robot_current_position:
                self.get_logger().info("position is LESS than current position")
                self.robot_current_position -= velocity
                feedback.current_position = self.robot_current_position
                self.get_logger().info("Trying to publish to feedback")

            #Otherwise, go forwards
            else: 
                self.get_logger().info("position is MORE than current position")
                self.robot_current_position += velocity
                feedback.current_position = self.robot_current_position
                self.get_logger().info("Trying to publish to feedback")
            self.get_logger().info("Sleeping")
            time.sleep(1)
            self.get_logger().info("After sleeping")
            

        else: 
            #Check if the robot is at the target position
            if self.robot_current_position == position:
                #Success
                self.get_logger().info(f"Success: robot is currently at {self.robot_current_position}")
                goal_handle.succeed()
            else:
                #Failure Abort!
                self.get_logger().error(f"Abort: robot is not at target position. It is at {self.robot_current_position}")
                goal_handle.abort()

            result.final_position = self.robot_current_position
            self.process_next_goal()
            return result
        
    def process_next_goal(self):
        if len(self.goal_queue) > 0:
            self.goal_queue.pop(0).execute()
        else:
            self.goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotServerNode() 
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()