#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# import eulertoquaternion and quaterniontoeuler services
from arduinobot_msgs.srv import EulerToQuaternion, QuaternionToEuler
from tf_transformations import quaternion_from_euler, euler_from_quaternion



class AngleConversionNode(Node): 
    def __init__(self):
        super().__init__("angle_conversion") 

        self.euler_to_quaternion = self.create_service(EulerToQuaternion, "euler_to_quaternion", self.eulerToQuaternionCallback)
        self.quaternion_to_euler = self.create_service(QuaternionToEuler, "quaternion_to_euler", self.quaternionToEulerCallback)
        self.get_logger().info("Angle Conversion Node has been started")

    def eulerToQuaternionCallback(self, request, response):
        self.get_logger().info("Received request for euler to quaternion conversion\n roll: %f\n pitch: %f\n yaw: %f\n" % (request.roll, request.pitch, request.yaw))
        response.x, response.y, response.z, response.w = quaternion_from_euler(request.roll, request.pitch, request.yaw)
        self.get_logger().info("Corresponding Quaternion: \n x: %f\n y: %f\n z: %f\n w: %f\n" % (response.x, response.y, response.z, response.w))
        return response

    def quaternionToEulerCallback(self, request, response):
        self.get_logger().info("Received request for quaternion to euler conversion\n x: %f\n y: %f\n z: %f\n w: %f\n" % (request.x, request.y, request.z, request.w))
        (response.roll, response.pitch, response.yaw) = euler_from_quaternion([request.x, request.y, request.z, request.w])
        self.get_logger().info("Corresponding Euler Angles: \n roll: %f\n pitch: %f\n yaw: %f\n" % (response.roll, response.pitch, response.yaw))
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AngleConversionNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()