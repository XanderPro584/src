# subscribe to the /mavros/global_position/local topic

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped





class LocalPoseSubNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("local_pose_sub") # MODIFY NAME
        self.get_logger().info("Node has been started")

        # Volatile Quality of Service profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # self.currentPos = self.create_subscription(
        #     Odometry, "/mavros/global_position/local", self.pose_cb, qos_profile=qos_profile,)
        
        self.currentPos = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.pose_cb, qos_profile=qos_profile,)
        
    def pose_cb(self, msg):
        # Log the current position and the heading
        current_position = msg.pose.position
        self.get_logger().info("Current position: " + str(current_position))


def main(args=None):
    rclpy.init(args=args)
    node = LocalPoseSubNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()