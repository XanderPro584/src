#Basic lifecycle node that creates a local reference frame and publishes the position to a ros topic
'''Lifecycle node that takes in the global position of the drone and publishes the local position and heading to a ros topic
    Subscribes to: /mavros/global_position/local
    Publishes to: /local_position
                  /local_heading
    '''
# Import Dependencies
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time
import numpy as np

# from PrintColours import *
from math import atan2, pow, sqrt, degrees, radians, sin, cos
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from example_interfaces.msg import Float64, Bool


from drone_interfaces.action import GoToWaypoint
from drone_interfaces.msg import LocalWaypoint

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading

from mavros_msgs.srv import CommandBool
from std_srvs.srv import Trigger

#Changing the subscriber to /mavros/local_position/pose

class DronePose(LifecycleNode):

    def __init__(self):
        super().__init__('drone_pose')
        self.get_logger().info('Drone Pose Node started')
        # INITIALIZE VARIABLES

        # Volatile Quality of Service profile
        self.volatile_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Reliable Quality of Service profile
        
        # LOCAL FRAME INITIALIZATION    
        self.frame_initialized = False
        self.current_pose_g = Odometry()
        self.current_heading_g = 0.0
        self.rotation_theta = 0.0
        self.gps_fix = False

        self.ready_to_initialize = False
        self.current_pose_list = []

        # WAYPOINT ACTION SERVER INITIALIZATION
        self.correction_vector_g = Pose()
        self.waypoint_g = PoseStamped()
        self.local_desired_heading_g = 0.0
        self.waypoint_server_is_activated = False

        self.goal_queue = []
        self.goal_lock = threading.Lock()
        self.goal_handle: ServerGoalHandle = None
        self.abort_current_goal = False

        self.ready_to_initialize_pub = self.create_publisher(
            Bool, "/ready_to_activate", qos_profile=10)

    # Lifecycle Node Functions
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring')

        # set up publishers and subscribers
        # Check for GPS fix, this message is not used for anything else
        self.gps_pos_check = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.gps_pos_check_cb, qos_profile=self.volatile_qos_profile) 

        self.local_position_pub = self.create_lifecycle_publisher(
            Point, "/local_position", qos_profile=self.volatile_qos_profile)
        
        self.current_heading_pub = self.create_lifecycle_publisher(
            Float64, "/local_heading", qos_profile=self.volatile_qos_profile)

        self.currentPos = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.pose_cb, qos_profile=self.volatile_qos_profile,)

        # Waypoint publisher
        self.local_pos_pub = self.create_lifecycle_publisher(
            PoseStamped, "/mavros/setpoint_position/local", qos_profile=self.volatile_qos_profile)
        
        # Server to cancel all goals in queue and the current goal
        self.cancel_all_goals_server = self.create_service(
            CommandBool, "/cancel_all_goals", self.cancel_all_goals_cb)
        
        # #Set homepoint service client
        # self.set_homepoint_client = self.create_client(
        #     Trigger, "/mavros/home_position/req_update")

        self.get_logger().info('Getting ready to initialize local frame ...')
        
        self.go_to_waypoint_server = ActionServer(
            self,
            GoToWaypoint,
            'go_to_waypoint',
            goal_callback=self.go_to_waypoint_goal_callback,
            handle_accepted_callback=self.go_to_waypoint_handle_accepted_callback,
            execute_callback=self.go_to_waypoint_execute_callback,
            cancel_callback=self.go_to_waypoint_cancel_callback,
            callback_group=ReentrantCallbackGroup(),)

        return TransitionCallbackReturn.SUCCESS
    
    def go_to_waypoint_goal_callback(self, goal_request: GoToWaypoint.Goal):
        # Use the check_waypoint_reached function to see if the goal waypoint is within 500 meters of the current position
        if self.check_waypoint_reached(pos_tol=500, angle_tol=10) \
            and self.waypoint_server_is_activated:
            # Accept the goal
            self.get_logger().info("Goal within max distance: ACCEPTED")
            return GoalResponse.ACCEPT
        else:
            # Reject the goal
            self.get_logger().info("Goal not within max distance or server is not activated: REJECTED")
            return GoalResponse.REJECT

    def go_to_waypoint_handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        # Add the goal to the queue
        with self.goal_lock:
            if self.goal_handle is not None:
                self.goal_queue.append(goal_handle)
                self.get_logger().info("Goal added to queue")
            else:
                goal_handle.execute()

    def go_to_waypoint_execute_callback(self, goal_handle: ServerGoalHandle):
        # Publish the waypoint to the local position topic
        with self.goal_lock:
            self.goal_handle = goal_handle

        # Log where the drone is going
        goal_position = goal_handle.request.goal_position
        self.get_logger().info("Going to {}".format(goal_position))
        self.set_destination(
            goal_position.x, 
            goal_position.y, 
            goal_position.z, 
            goal_position.psi)

        local_position = LocalWaypoint()
        result = GoToWaypoint.Result()

        while self.check_waypoint_reached() == 0:
            current_pos = self.get_local_position()
            local_position.x = current_pos.x #Turn Position() into LocalWaypoint()
            local_position.y = current_pos.y
            local_position.z = current_pos.z
            local_position.psi = self.current_heading_g

            # Check if the goal has been cancelled
            if not goal_handle.is_active or self.abort_current_goal or self.waypoint_server_is_activated == False:
                self.abort_current_goal = False
                self.get_logger().info("Goal aborted")
                goal_handle.abort()
                result.final_position = local_position
                self.process_next_goal_in_queue()
                return result
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.cancel_goal()
                self.get_logger().info("Goal cancelled")
                result.final_position = local_position
                self.process_next_goal_in_queue()
                return result

        
            # Send feedback of the current position
            feedback_msg = GoToWaypoint.Feedback()
            feedback_msg.current_position = local_position
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        else: 
            # Send the result of the action
            goal_handle.succeed()
            self.get_logger().info("Goal reached")
            result.final_position = local_position
            self.process_next_goal_in_queue()
            return result

    def go_to_waypoint_cancel_callback(self, goal_handle: ServerGoalHandle):
        # Cancel the goal
        self.get_logger().info("Received Cancel Request")
        return CancelResponse.ACCEPT
    
    def cancel_goal(self):
        # Cancel the goal
        self.get_logger().info("Sending coordinate of current position as waypoint")
        current_pos_as_local = self.get_local_position()
        self.set_destination(
            current_pos_as_local.x,
            current_pos_as_local.y,
            current_pos_as_local.z,
            self.current_heading_g
        )
        time.sleep(1)
        
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up')

        # self.local_position_pub.destroy()
        # self.current_heading_pub.destroy()
        # self.currentPos.destroy()
        # self.local_pos_pub.destroy()
        # self.gps_pos_check.destroy()
        # self.ready_to_initialize_pub.destroy()
        # self.go_to_waypoint_server.destroy()

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating')
        self.get_logger().info('Initializing local frame ...')
        self.initialize_local_frame()
        if not self.frame_initialized:
            return TransitionCallbackReturn.ERROR
        else: 
            self.waypoint_server_is_activated = True
            return super().on_activate(state)
    
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating')
        self.current_heading_g = 0.0
        self.rotation_theta = 0.0
        self.current_pose_list = []

        self.frame_initialized = False
        self.gps_fix = False
        self.goal_queue = []
        self.goal_handle = None
        self.waypoint_server_is_activated = False

        return super().on_deactivate(state)
    
    def process_next_goal_in_queue(self):
        with self.goal_lock:
            if len(self.goal_queue) > 0:
                self.goal_queue.pop(0).execute()
            else:
                self.goal_handle = None

    # GO TO WAYPOINT ACTION SERVER CALLBACK FUNCTIONS
    def set_heading(self, heading):
        self.local_desired_heading_g = heading
        heading = heading + self.rotation_theta
        # self.get_logger().info("The desired heading is {}".format(self.local_desired_heading_g))
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

        return orientation


    def set_destination(self, x, y, z, psi):
        orientation = self.set_heading(psi)

        original_point = self.transform_point(x, y)

        position = Point()
        position.x = original_point[0]
        position.y = original_point[1]
        position.z = z

        self.get_logger().info("Destination set to x:{} y:{} z:{} origin frame".format(position.x, position.y, position.z))

        self.waypoint_g.pose.orientation = orientation
        self.waypoint_g.pose.position = position
        self.local_pos_pub.publish(self.waypoint_g)

    # def rotate_point(self, x, y, theta):
    #     rotation_matrix = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
    #     point = np.array([x, y])
    #     rotated_point = np.dot(rotation_matrix, point)
    #     return rotated_point
    
    def transform_point(self, x, y):
        # Apply rotation first and then translation
        theta = radians(self.rotation_theta - 90)
        rotation_matrix = np.array([[cos(theta), -sin(theta)],
                                    [sin(theta), cos(theta)]])
        rotated_point = np.dot(rotation_matrix, np.array([x, y]))

        translated_point = rotated_point + np.array([self.translation_x, self.translation_y])

        return translated_point
    
    def inverse_transform_point(self, x, y):
        # Apply inverse translation first and then inverse rotation
        inverse_translated_point = np.array([x, y]) - np.array([self.translation_x, self.translation_y])

        theta = radians(-(self.rotation_theta - 90))
        inverse_rotation_matrix = np.array([[cos(theta), -sin(theta)],
                                           [sin(theta), cos(theta)]])
        inverse_rotated_point = np.dot(inverse_rotation_matrix, inverse_translated_point)

        return inverse_rotated_point
    
    # def inverse_rotate_point(self, x, y, theta):
    #     inverse_rotation_matrix = np.array([[cos(theta), sin(theta)], [-sin(theta), cos(theta)]])
    #     point = np.array([x, y])
    #     rotated_point = np.dot(inverse_rotation_matrix, point)
    #     return rotated_point

    def check_waypoint_reached(self, pos_tol=0.5, angle_tol=0.08):
        dx = abs(self.waypoint_g.pose.position.x - self.current_pose_g.pose.position.x)
        dy = abs(self.waypoint_g.pose.position.y - self.current_pose_g.pose.position.y)
        dz = abs(self.waypoint_g.pose.position.z - self.current_pose_g.pose.position.z)

        dMag = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))

        # Convert quaternion to euler angles for both current and desired orientations
        current_euler = self.quaternion_to_euler(self.current_pose_g.pose.orientation)
        desired_euler = self.quaternion_to_euler(self.waypoint_g.pose.orientation)

        # Extract yaw angles from euler angles
        current_yaw = current_euler[2]  # Yaw angle is at index 2
        desired_yaw = desired_euler[2]

        # Calculate angular difference between current and desired yaw angles
        yaw_angle_diff = abs(current_yaw - desired_yaw)
        self.get_logger().info("Yaw angle difference: {}".format(yaw_angle_diff))
        self.get_logger().info("dMag: {}".format(dMag))

        if dMag < pos_tol and yaw_angle_diff < angle_tol:
            return 1
        else:
            return 0    

    def quaternion_to_euler(self, q):
        # Convert quaternion to euler angles
        t0 = +2.0 * (q.w * q.x + q.y * q.z)
        t1 = +1.0 - 2.0 * (q.x**2 + q.y**2)
        roll_x = np.arctan2(t0, t1)

        t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)

        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y**2 + q.z**2)
        yaw_z = np.arctan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians


    def cancel_all_goals_cb(self, request, response):
        self.get_logger().info("Cancel all goals request received")
        self.abort_current_goal = True
        self.goal_queue = []
        self.goal_handle = None
        return response
    
    # LOCAL FRAME CALLBACK FUNCTIONS
    def pose_cb(self, msg):
        self.current_pose_g = msg

        if self.gps_fix:
            if len(self.current_pose_list) >= 10:
                if self.ready_to_initialize == False:
                    self.get_logger().info("Ready to initialize")
                    self.ready_to_initialize = True

                self.current_pose_list.pop(0)
                self.ready_to_initialize_pub.publish(Bool(data=True))


            self.current_pose_list.append(self.current_pose_g)

        q0, q1, q2, q3 = (
            self.current_pose_g.pose.orientation.w,
            self.current_pose_g.pose.orientation.x,
            self.current_pose_g.pose.orientation.y,
            self.current_pose_g.pose.orientation.z,
        )

        psi = atan2((2 * (q0 * q3 + q1 * q2)),
                    (1 - 2 * (pow(q2, 2) + pow(q3, 2))))
        
        self.current_heading_g = degrees(psi) - self.rotation_theta
        if self.frame_initialized:
            self.current_heading_pub.publish(Float64(data=self.current_heading_g))
            local_position = self.get_local_position()
            self.local_position_pub.publish(local_position)

    def gps_pos_check_cb(self, msg):
        if self.gps_fix == False:
            self.gps_fix = True      
            self.get_logger().info("GPS fix acquired")  

    def get_local_position(self):
        x = self.current_pose_g.pose.position.x
        y = self.current_pose_g.pose.position.y
        z = self.current_pose_g.pose.position.z
        
        rotated_point = self.inverse_transform_point(x, y)

        position = Point()
        position.x = rotated_point[0]
        position.y = rotated_point[1]
        position.z = z

        return position

    def initialize_local_frame(self):
        # self.set_homepoint()
        self.rotation_theta = 0.0
        self.translation_x = 0.0
        self.translation_y = 0.0

        if len(self.current_pose_list) == 10:
            for previous_pose in self.current_pose_list:
                q0, q1, q2, q3 = (
                    previous_pose.pose.orientation.w,
                    previous_pose.pose.orientation.x,
                    previous_pose.pose.orientation.y,
                    previous_pose.pose.orientation.z,
                )

                psi = atan2((2 * (q0 * q3 + q1 * q2)),
                            (1 - 2 * (pow(q2, 2) + pow(q3, 2))))

                self.rotation_theta += degrees(psi)

                self.translation_x += previous_pose.pose.position.x
                self.translation_y += previous_pose.pose.position.y

            self.rotation_theta /= 10.0
            self.translation_x /= 10.0
            self.translation_y /= 10.0

            self.get_logger().info("coordinate offset set")        
            self.get_logger().info("The X-Axis is facing: {}".format(self.rotation_theta))
            self.get_logger().info("The coordinate offset is: ({}, {})".format(self.translation_x, self.translation_y))
            self.frame_initialized = True

        else:
            self.get_logger().info("Not enough data to initialize local frame")
            self.frame_initialized = False

    # def set_homepoint(self):
    #     self.get_logger().info("Setting homepoint")
    #     self.set_homepoint_client.wait_for_service()
    #     request = Trigger.Request()
    #     future = self.set_homepoint_client.call_async(request)
    #     rclpy.spin_until_future_complete(self, future)
    #     response = future.result()
    #     if response.success:
    #         self.get_logger().info("Homepoint set")
    #     else:
    #         self.get_logger().info("Failed to set homepoint")



def main(args=None):
    rclpy.init(args=args)
    drone_pose = DronePose()
    rclpy.spin(drone_pose, MultiThreadedExecutor())
    drone_pose.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


    
        

        
