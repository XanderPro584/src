#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time

# from PrintColours import *
from math import atan2, pow, sqrt, degrees, radians, sin, cos
from math import atan2, pow, sqrt, degrees, radians, sin, cos
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL, CommandLong, CommandBool, SetMode
from example_interfaces.msg import Float64



class DroneControllerNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("drone_controller") # MODIFY NAME

        self.get_logger().info("Drone Controller Node has been started")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.current_state_g = State()
        self.current_pose_g = Odometry()
        self.correction_vector_g = Pose()
        self.local_offset_pose_g = Point()
        self.waypoint_g = PoseStamped()

        self.current_heading_g = 0.0
        self.local_offset_g = 0.0
        self.correction_heading_g = 0.0
        self.local_desired_heading_g = 0.0

        self.local_position_pub = self.create_publisher(Point, "/local_position", qos_profile=qos_profile)
        self.current_heading_pub = self.create_publisher(Float64, "/local_heading", qos_profile=qos_profile)
        self.currentPos = self.create_subscription(Odometry, "/mavros/global_position/local", self.pose_cb, qos_profile=qos_profile)
        self.state_sub = self.create_subscription(State, "/mavros/state", self.state_cb, qos_profile=qos_profile)
        self.local_pos_pub = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", qos_profile=qos_profile)

        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.land_client = self.create_client(CommandTOL, "/mavros/cmd/land")
        self.takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.command_client = self.create_client(CommandLong, "/mavros/cmd/command")

        self.get_logger().info("Drone Controller Node has been initialized")

        self.initialize_local_frame()

    def state_cb(self,msg):
        self.current_state_g = msg

    def pose_cb(self, msg):
        self.current_pose_g = msg
        self.enu_2_local()

        q0, q1, q2, q3 = (
            self.current_pose_g.pose.pose.orientation.w,
            self.current_pose_g.pose.pose.orientation.x,
            self.current_pose_g.pose.pose.orientation.y,
            self.current_pose_g.pose.pose.orientation.z,
        )

        psi = atan2((2 * (q0 * q3 + q1 * q2)),
                    (1 - 2 * (pow(q2, 2) + pow(q3, 2))))
        
        self.current_heading_g = degrees(psi) - self.local_offset_g
        # self.get_logger().info("current_heading_g: {}".format(self.current_heading_g))
        self.current_heading_pub.publish(Float64(data=self.current_heading_g))

    def enu_2_local(self):
        x, y, z = (
            self.current_pose_g.pose.pose.position.x,
            self.current_pose_g.pose.pose.position.y,
            self.current_pose_g.pose.pose.position.z,
        )

        current_pos_local = Point()

        current_pos_local.x = x * cos(radians((self.local_offset_g - 90))) - y * sin(
            radians((self.local_offset_g - 90))
        )

        current_pos_local.y = x * sin(radians((self.local_offset_g - 90))) + y * cos(
            radians((self.local_offset_g - 90))
        )

        current_pos_local.z = z
        # self.get_logger().info("current_pos_local: {}".format(current_pos_local))
        self.local_position_pub.publish(current_pos_local)
        return current_pos_local

    def initialize_local_frame(self):
        self.local_offset_g = 0.0

        for i in range(30):
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

            q0, q1, q2, q3 = (
                self.current_pose_g.pose.pose.orientation.w,
                self.current_pose_g.pose.pose.orientation.x,
                self.current_pose_g.pose.pose.orientation.y,
                self.current_pose_g.pose.pose.orientation.z,
            )

            psi = atan2((2 * (q0 * q3 + q1 * q2)),
                        (1 - 2 * (pow(q2, 2) + pow(q3, 2))))

            self.local_offset_g += degrees(psi)
            self.local_offset_pose_g.x += self.current_pose_g.pose.pose.position.x
            self.local_offset_pose_g.y += self.current_pose_g.pose.pose.position.y
            self.local_offset_pose_g.z += self.current_pose_g.pose.pose.position.z

        self.local_offset_pose_g.x /= 30.0
        self.local_offset_pose_g.y /= 30.0
        self.local_offset_pose_g.z /= 30.0
        self.local_offset_g /= 30.0

        self.get_logger().info("coordinate offset set")        
        self.get_logger().info("The X-Axis is facing: {}".format(self.local_offset_g))



    #     self.local_offset_g = 0.0
    #     self.local_offset_pose_g = Point()
    #     self.local_pos_pub = self.create_publisher(
    #         PoseStamped, "/local_position", 10)
        
    #     self.global_pos_sub = self.create_subscription(
    #         Odometry, "/mavros/global_position/local", self.gps_callback, qos_profile=qos_profile)
    # def pose_cb(self, msg):
    #     self.get_logger().info("called pose_cb")
    #     self.local_offset_g, self.local_offset_pose_g = self.calculate_local_offset(msg)

    # def calculate_local_offset(self, msg):
    #     self.get_logger().info("called calculate_local_offset")
    #     q0, q1, q2, q3 = (
    #         msg.pose.pose.orientation.w,
    #         msg.pose.pose.orientation.x,
    #         msg.pose.pose.orientation.y,
    #         msg.pose.pose.orientation.z,
    #     )

    #     psi = atan2((2 * (q0 * q3 + q1 * q2)),
    #                 (1 - 2 * (pow(q2, 2) + pow(q3, 2))))

    #     local_offset_g = degrees(psi)

    #     local_offset_pose = Point()
    #     local_offset_pose.x = msg.pose.pose.position.x
    #     local_offset_pose.y = msg.pose.pose.position.y
    #     local_offset_pose.z = msg.pose.pose.position.z

    #     return local_offset_g, local_offset_pose

    # def gps_callback(self, msg):
    #     self.get_logger().info("called gps_callback")

    #     self.pose_cb(msg)
    #     global_x = msg.pose.pose.position.x
    #     global_y = msg.pose.pose.position.y
    #     global_z = msg.pose.pose.position.z

    #     local_x = global_x * cos(radians(self.local_offset_g)) - global_y * sin(radians(self.local_offset_g))
    #     local_y = global_x * sin(radians(self.local_offset_g)) + global_y * cos(radians(self.local_offset_g))
    #     local_z = global_z



    #     local_pose = PoseStamped()
    #     local_pose.header.stamp = self.get_clock().now().to_msg()
    #     local_pose.pose.position.x = local_x
    #     local_pose.pose.position.y = local_y
    #     local_pose.pose.position.z = local_z

    #     local_pose.pose.orientation.x = 0.0
    #     local_pose.pose.orientation.y = 0.0
    #     local_pose.pose.orientation.z = 0.0
    #     local_pose.pose.orientation.w = 1.0        
        
    #     self.local_pos_pub.publish(local_pose)


def main(args=None):
    rclpy.init(args=args)
    node = DroneControllerNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()