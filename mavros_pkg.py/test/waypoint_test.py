import rclpy
from rclpy.node import Node
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint
from mavros_msgs.msg import WaypointList
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandInt


class WaypointPushNode(Node):
    def __init__(self):
        super().__init__('waypoint_push_node')
        self.waypoint_push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.start_mission_client = self.create_client(CommandInt, '/mavros/cmd/command_int')

        while not self.arm_client.wait_for_service(1):
            self.get_logger().warn("Waiting for server to start")
        self.arm_request = CommandBool.Request()
        self.arm_request.value = True
        self.arm_client.call_async(self.arm_request)
        
        # See /mavros/mavros_msgs/msg/Waypoint.msg and /mavros/mavros_msgs/msg/WaypointList.msg
        # waypoint_list = WaypointList()

        self.waypoint = Waypoint()
        self.waypoint.frame = 6 # frame 6 = Global relative altitude (to home position)
        self.waypoint.command = 16 # command 16 = MAV_CMD_NAV_WAYPOINT -> go to waypoint
        self.waypoint.is_current = False
        self.waypoint.autocontinue = True
        self.waypoint.param1 = float(0) # command 16 -> Hold time
        self.waypoint.param2 = float(0) # command 16 -> Accept Radius (how close you need to be to mark waypoint as reached)
        self.waypoint.param3 = float(0.5) # command 16 -> Pass Radius (set the radius of a turn around a waypoint, positive=clockwise orbit; negative=counter-clockwise orbit)
        self.waypoint.param4 = float(0.2) # command 16 -> Yaw Heading; NaN = current system yaw heading mode (face next waypoint usually)
        self.get_logger().info(f"{self.waypoint._param3}")
        self.waypoint.x_lat = float(-35.3632622)
        self.waypoint.y_long = float(149.1652375)
        self.waypoint.z_alt = float(10)

        while not self.waypoint_push_client.wait_for_service(1):
            self.get_logger().warn("Waiting for server to start")
        # Populate your request
        self.waypoint_push_request = WaypointPush.Request() 
        # See /mavros/mavros_msgs/srv/WaypointPush.srv
        self.waypoint_push_request.waypoints = [self.waypoint]
        # Make service call and store service response to check for success and number of waypoints transferred
        self.get_logger().info("attempting to push request")

        self.waypoint_push_client.call(self.waypoint_push_request)

        while not self.start_mission_client.wait_for_service(1):
            self.get_logger().warn("Waiting for server to start")

        self.start_mission_request = CommandInt.Request()
        self.start_mission_request.command = 300
        self.start_mission_client.call(self.start_mission_request)


def main(args=None):
    rclpy.init(args=args)
    waypoint_push = WaypointPushNode()
    rclpy.spin(waypoint_push)
    waypoint_push.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()