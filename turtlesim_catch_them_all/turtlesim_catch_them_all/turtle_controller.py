#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from functools import partial
import math
from turtle_interfaces.msg import TurtleInfoArray, TurtleInfo
from turtle_interfaces.srv import EatTurtle
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Vector3

class TurtleControllerNode(Node): # MODIFY NAME
    
    def __init__(self):
        super().__init__("turtle_controller") # MODIFY NAME
        self.get_logger().info("Turtle Controller node has started!!!")

        self.declare_parameter("ctrl_frequency", 100)

        self.turtles = []

        self.turtle1_pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.update_turtle1_pose, 10)
        self.alive_turtles_sub = self.create_subscription(TurtleInfoArray, "alive_turtles", self.update_alive_turtles, 10)
        self.turtle_ctrl_pub = self.create_publisher(Twist, "turtle1/cmd_vel", 10)

        ctrl_frequency = 1 / self.get_parameter("ctrl_frequency").value
        self.timer = self.create_timer(ctrl_frequency, self.publish_turtle_ctrl)

        # self.ctrl_timer = self.create_timer(0.01, self.publish_turtle_ctrl)
    
    def publish_turtle_ctrl(self):
        if len(self.turtles):
            target_turtle = self.find_target_turtle()
            angle_from_target = self.find_angle(target_turtle)
            # self.get_logger().info(f"Angle to closest turtle is {angle_from_target} degrees")

            cmd_vel = Twist()

            cmd_vel.linear =  self.find_linear_vector(target_turtle)
            cmd_vel.angular = self.find_angular_vector(angle_from_target)

            self.turtle_ctrl_pub.publish(cmd_vel)
            self.check_if_eaten(target_turtle)

    def check_if_eaten(self, target_turtle):
        if self.get_distance(target_turtle) < .3:
            self.call_eat_turtle_server(target_turtle)


    def find_linear_vector(self,target_turtle):
        dist = self.get_distance(target_turtle)
        linear_vector = Vector3()
        linear_vector.x = dist

        return linear_vector


    def find_angular_vector(self, angle_from_target):
        turtle1_angle = math.degrees(self.turtle1_pose.theta)
        if turtle1_angle < 0:
            turtle1_angle += 360
        change_in_ang = 0

        if angle_from_target - turtle1_angle < 180:
            change_in_ang = angle_from_target - turtle1_angle

        else: 
            change_in_ang = (angle_from_target - 360 - turtle1_angle)
        angular_rate = (change_in_ang/ 360)*10

        angular_vector = Vector3()
        angular_vector.z = angular_rate

        self.get_logger().info(f"turtle1 ange: {turtle1_angle}\nAngle from targe: {angle_from_target}\nchange in angle: {change_in_ang}")

        return angular_vector

        



    def find_angle(self, turtle):
        delta_x = turtle.x - self.turtle1_pose.x
        delta_y = turtle.y - self.turtle1_pose.y

        angle = math.atan2(delta_y, delta_x)
        angle = math.degrees(angle)

        if angle < 0:
            angle += 360
        return angle

    def find_target_turtle(self):
        new_min_dist = self.get_distance(self.turtles[0])
        new_closest_turtle = self.turtles[0]

        for turtle in self.turtles:
            turtle_dist = self.get_distance(turtle)
            if turtle_dist < new_min_dist:
                new_min_dist = turtle_dist
                new_closest_turtle = turtle

        # self.get_logger().info(f"Closest turtle was at ({new_closest_turtle.x},{new_closest_turtle.y}), {new_min_dist} from turtle1")
        return new_closest_turtle

    def get_distance(self, turtle):
            x_dist = abs(self.turtle1_pose.x - turtle.x)
            y_dist = abs(self.turtle1_pose.y - turtle.y)
            total_dist = (y_dist**2 + x_dist**2)**(1/2)

            return total_dist


        
        


    # def decide_turtle_to_eat(self):
    #     if len(self.turtles):
    #         self.call_eat_turtle_server(self.turtles[0])
    #         self.get_logger().info(f"trying to eat {self.turtles[0]}")

    #     else: 
    #         self.get_logger().info("there isn't a turtle in first place yet")

    def call_eat_turtle_server(self, turtle):
        client = self.create_client(EatTurtle, "eat_turtle")
        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for Server Eat Turtle...")

        request = EatTurtle.Request()
        request.turtle = turtle

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_eat_turtle, turtle=turtle))

    def callback_call_eat_turtle(self, future, turtle):
        try: 
            response = future.result()
            self.get_logger().info(f"Eating {turtle.name} was {response.success}")

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
    

    def update_turtle1_pose(self, pose):

        self.turtle1_pose = pose
        
        # self.get_logger().info(f"turtle 1 pose is {self.turtle1_pose}")

    def update_alive_turtles(self, turtle_info_array):
        self.turtles = turtle_info_array.turtles
        self.num_turtles = len(self.turtles)

        # self.get_logger().info(f"There are {self.num_turtles} turtles alive.")




def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()