#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import CountUntil 
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup



class CountUntilSeverNode(Node): 
    def __init__(self):
        super().__init__("count_until_server") 
        #Action server initialization
        self.goal_handle: ServerGoalHandle = None
        self.goal_lock = threading.Lock()
        self.goal_queue = []

        self.count_until_server = ActionServer(self, 
            CountUntil, 
            "count_until", 
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback, 
            callback_group=ReentrantCallbackGroup()
            ) 

        self.get_logger().info("Count Until Server has been started")



    def handle_accepted_callback(self, goal_handle: ServerGoalHandle): 
        with self.goal_lock:
            if self.goal_handle is not None:
                self.goal_queue.append(goal_handle)
            else: 
                goal_handle.execute()




    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info(f"Received a goal request: {goal_request}")

        #Policy: refuse new goal if current goal is still active
        # with self.goal_lock:
        #     if self.goal_handle is not None and self.goal_handle.is_active:
        #         self.get_logger().info("A goal is already active, rejecting new goal")
        #         return GoalResponse.REJECT 

        #Validate the goal request
        if goal_request.target_number < 1:
            self.get_logger().info("Goal request rejected")
            return GoalResponse.REJECT
        
        #Policy: preempt existing goal when receiving new goal
        # with self.goal_lock: 
        #     if self.goal_handle is not None and self.goal_handle.is_active:
        #         self.get_logger().info("Abort current goal and accept new goal")
        #         self.goal_handle.abort()

        self.get_logger().info("Goal request accepted")
        return GoalResponse.ACCEPT
    

    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a cancel request")
        return CancelResponse.ACCEPT #ACCEPT or REJECT
    


    def execute_callback(self, goal_handle: ServerGoalHandle): 
        with self.goal_lock:
            self.goal_handle = goal_handle
        #Get request from goal 
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period

        #Execute action
        self.get_logger().info(f"Goal target number: {target_number}")
        self.get_logger().info(f"Goal period: {period}")

        result = CountUntil.Result()
        feedback = CountUntil.Feedback()
        
        counter = 0
        for i in range(target_number):
            if not goal_handle.is_active:
                result.reached_number = counter
                self.process_next_goal_in_queue()
                return result

            if goal_handle.is_cancel_requested:
                self.get_logger().info("Canceling the goal")
                #Return final state
                goal_handle.canceled() 
                result.reached_number = counter
                self.process_next_goal_in_queue()
                return result
            
            counter += 1

            self.get_logger().info(f"Counter: {counter}")
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            time.sleep(period)

        #Once done, set goal final state
        goal_handle.succeed()

        #send the result
        result.reached_number = counter
        self.process_next_goal_in_queue()
        return result
    

    
    def process_next_goal_in_queue(self):
        with self.goal_lock:
            if len(self.goal_queue) > 0:
                self.goal_queue.pop(0).execute()
            else: 
                self.goal_handle = None

        

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilSeverNode() 
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()