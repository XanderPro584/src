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
        # Initialization
        self.current_state_g = State()
        self.current_pose_g = Odometry()
        self.correction_vector_g = Pose()
        self.local_offset_pose_g = Point()
        self.waypoint_g = PoseStamped()

        self.current_heading_g = 0.0
        self.local_offset_g = 0.0
        self.correction_heading_g = 0.0
        self.local_desired_heading_g = 0.0

        self.frame_initialized = False

        # set up publishers and subscribers
        self.local_position_pub = self.create_publisher(Point, "/local_position", qos_profile=qos_profile)
        self.current_heading_pub = self.create_publisher(Float64, "/local_heading", qos_profile=qos_profile)
        self.currentPos = self.create_subscription(Odometry, "/mavros/global_position/local", self.pose_cb, qos_profile=qos_profile)
        self.state_sub = self.create_subscription(State, "/mavros/state", self.state_cb, qos_profile=qos_profile)
        self.local_pos_pub = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", qos_profile=qos_profile)

        # self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        # self.land_client = self.create_client(CommandTOL, "/mavros/cmd/land") 
        # self.takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        # self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        # self.command_client = self.create_client(CommandLong, "/mavros/cmd/command")

        self.get_logger().info("Drone Controller Node has been initialized")
        
        # Connect then initialize local frame
        if self.wait4connect() != -1:
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
        # self.get_logger().info(f"current_pose_g: {self.current_pose_g.pose.pose.orientation}")
        # self.get_logger().info(f"local_offset_g: {self.local_offset_g}")

        psi = atan2((2 * (q0 * q3 + q1 * q2)),
                    (1 - 2 * (pow(q2, 2) + pow(q3, 2))))
        
        self.current_heading_g = degrees(psi) - self.local_offset_g
        # self.get_logger().info("current_heading_g: {}".format(self.current_heading_g))
        if self.frame_initialized:
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
        if self.frame_initialized:
            self.local_position_pub.publish(current_pos_local)
        return current_pos_local

    def initialize_local_frame(self):
        self.local_offset_g = 0.0

        for i in range(60):
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

        self.local_offset_pose_g.x /= 60.0
        self.local_offset_pose_g.y /= 60.0
        self.local_offset_pose_g.z /= 60.0
        self.local_offset_g /= 60.0

        self.get_logger().info("coordinate offset set")        
        self.get_logger().info("The X-Axis is facing: {}".format(self.local_offset_g))
        self.frame_initialized = True


    def wait4connect(self):
        self.get_logger().info("Waiting for connection...")
        while not self.current_state_g.connected:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        else: 
            if self.current_state_g.connected:
                self.get_logger().info("Connected to FCU")
            else:
                self.get_logger().info("Connection failed")
                return -1

def main(args=None):
    rclpy.init(args=args)
    node = DroneControllerNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()