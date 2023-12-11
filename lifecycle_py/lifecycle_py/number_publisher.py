#object oriented ROS2 node called NumberPublisherNode that has a one second timer and publishes a number that counts up. It publishes that number every second to the ros topic /number
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64


class NumberPUblisherNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("number_publisher") # MODIFY NAME
        self.counter = 0
        self.create_timer(1, self.timer_callback)
        self.publisher_ = self.create_publisher(Int64, "/number", 10)
        self.get_logger().info("Number Publisher has been started")

    def timer_callback(self):
        msg = Int64()
        msg.data = self.counter
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = NumberPUblisherNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()