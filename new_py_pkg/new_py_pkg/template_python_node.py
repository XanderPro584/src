#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget


class MyCustomNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("node_name") # MODIFY NAME
        # self.position_publisher = self.create_publisher(PositionTarget, '/mavros/setpoint_position/local', 10)
        # self.takeoff_command = PositionTarget()
        # self.takeoff_command.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        # self.takeoff_command.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ
        # self.takeoff_command.position.z = 10

        # self.timer_ = self.create_timer(0.1, self.publish_position)

    # def publish_position(self):
        # self.position_publisher.publish(self.takeoff_command)


def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()