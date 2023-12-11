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

        self.set_waypoint_server = self.create_service(CommandLong, "/mavros/cmd/command", self.set_waypoint_cb)

        self.get_logger().info("Drone Controller Node has been initialized")

        # Connect then initialize local frame
        self.wait4connect()

        self.wait4start()

        self.initialize_local_frame()

    def set_waypoint_cb(self, request, response):
        # check that drone is in guided mode
        if self.current_state_g.mode == "GUIDED":
            self.set_destination(request.param5, request.param6, request.param7, request.param4)
            response.success = True
            return response
        else:
            response.success = False
            return response
    
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
            
    def wait4start(self):
        self.get_logger().info("Waiting for user to set mode to GUIDED")
        while  self.current_state_g.mode != "GUIDED":
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        else:
            if self.current_state_g.mode == "GUIDED":
                self.get_logger().info("Mode set to GUIDED. Starting Mission...")
                return 0
            else:
                self.get_logger().info("Error startting mission")
                return -1
        
            
    def set_heading(self, heading):
        self.local_desired_heading_g = heading
        heading = heading + self.correction_heading_g + self.local_offset_g

        self.get_logger().info("The desired heading is {}".format(self.local_desired_heading_g))

        yaw = radians(heading)
        pitch = 0.0
        roll = 0.0

        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)

        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)

        qw = cy * cr * cp + sy * sr * sp
        qx = cy * sr * cp - sy * cr * sp
        qy = cy * cr * sp + sy * sr * cp
        qz = sy * cr * cp - cy * sr * sp

        orientation = Quaternion()
        orientation.x = qx
        orientation.y = qy
        orientation.z = qz
        orientation.w = qw

        self.waypoint_g.pose.orientation = orientation

    def set_destination(self, x, y, z, psi):
        self.set_heading(psi)

        theta = radians((self.correction_heading_g + self.local_offset_g - 90))

        Xlocal = x * cos(theta) - y * sin(theta)
        Ylocal = x * sin(theta) + y * cos(theta)
        Zlocal = z

        x = Xlocal + self.correction_vector_g.position.x + self.local_offset_pose_g.x
        y = Ylocal + self.correction_vector_g.position.y + self.local_offset_pose_g.y
        z = Zlocal + self.correction_vector_g.position.z + self.local_offset_pose_g.z
        
        self.get_logger().info("Destination set to x:{} y:{} z:{} origin frame".format(x, y, z))

        position = Point()
        position.x = x
        position.y = y
        position.z = z

        self.waypoint_g.pose.position = position
        self.local_pos_pub.publish(self.waypoint_g)

    def check_waypoint_reached(self, pos_tol=0.3, head_tol=0.01):
        self.local_pos_pub.publish(self.waypoint_g)

        dx = abs(self.waypoint_g.pose.position.x - self.current_pose_g.pose.pose.position.x)
        dy = abs(self.waypoint_g.pose.position.y - self.current_pose_g.pose.pose.position.y)
        dz = abs(self.waypoint_g.pose.position.z - self.current_pose_g.pose.pose.position.z)

        dMag = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))

        cosErr = cos(radians(self.current_heading_g)) - cos(
            radians(self.local_desired_heading_g)
        )

        sinErr = sin(radians(self.current_heading_g)) - sin(
            radians(self.local_desired_heading_g)
        )

        dHead = sqrt(pow(cosErr, 2) + pow(sinErr, 2))

        if dMag < pos_tol and dHead < head_tol:
            return 1
        else:
            return 0

def main(args=None):
    rclpy.init(args=args)
    node = DroneControllerNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()