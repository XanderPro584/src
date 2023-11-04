#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from turtle_interfaces.msg import TurtleInfoArray, TurtleInfo
from turtle_interfaces.srv import EatTurtle
from turtlesim.srv import Spawn, Kill

import random as rand
from functools import partial


class TurtleSpawnerNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("turtle_spawner") # MODIFY NAME
        self.get_logger().info("Turtle Spawner has started")

        self.declare_parameter("spawn_frequency", 2)
        spawn_frequency= 1 / self.get_parameter("spawn_frequency").value

        self.turtles = []

        self.alive_turtles_pub = self.create_publisher(TurtleInfoArray, "alive_turtles", 10)
        self.new_turtle_timer = self.create_timer(2, self.create_new_turtle)
        self.pub_turtle_info_array_timer = self.create_timer(1, self.pub_turtle_info_array)
        self.eat_turtle_service = self.create_service(EatTurtle, "eat_turtle", self.eat_turtle)

    def pub_turtle_info_array(self):
        turtle_info_array = TurtleInfoArray()
        turtle_info_array.turtles = self.turtles
        self.alive_turtles_pub.publish(turtle_info_array)

    def eat_turtle(self, request, response):
        turtle = request.turtle

        if turtle in self.turtles:
            self.call_kill_server(turtle)

            self.turtles.remove(turtle)
            self.get_logger().info(f"Removing {turtle.name} from Alive Turtles")
            self.pub_turtle_info_array()
            
            response.success = True
            return response
        else:
            self.get_logger().error("This turtle is not alive or doesn't exist")
            response.success = False
            return response

    def call_kill_server(self, turtle):
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for Kill Server...")

        request = Kill.Request()
        request.name = turtle.name

        client.call_async(request)




    def create_new_turtle(self):
        x = float(rand.randint(1, 10))
        y = float(rand.randint(1, 11))
        theta = float(rand.randint(0, 360))

        self.spawn_turtle(x, y, theta)
            
    def spawn_turtle(self, x, y, theta):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for Spawn Server...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_spawn_turtle, x=x, y=y, theta = theta))

    def callback_spawn_turtle(self, future, x, y, theta):
        try: 
            response = future.result()
            self.get_logger().info(f"spawned {response.name} at ({x},{y})")

            turtle = TurtleInfo()
            turtle.x = x
            turtle.y = y
            turtle.name = response.name

            self.turtles.append(turtle)

        except Exception as e:
            self.get_logger().error("Spawn failed %r" % (e,))



def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()