#object oriented ROS2 node called NumberPublisherNode that has a one second timer and publishes a number that counts up. It publishes that number every second to the ros topic /number
#!/usr/bin/env python3
import rclpy
# from rclpy.node import Node

from std_msgs.msg import Int64
#lifecycle dependencies 
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn


class NumberPUblisherNode(LifecycleNode): 
    def __init__(self):
        super().__init__("number_publisher") 
        self.get_logger().info("IN constructor")
        self.counter_ = 0
        self.number_publisher_ = None
        self.number_timer_ = None


    #Create ROS2 communications, connect to HW 
    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_configure")
        self.number_timer_ = self.create_timer(1, self.publish_number)
        self.number_publisher_ = self.create_lifecycle_publisher(Int64, "/number", 10) #Create lifecycle publisher
        self.number_timer_.cancel() #cancel the timer
        # raise Exception()
        return TransitionCallbackReturn.SUCCESS

    
    
    #Enable HW
    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_activate")
        self.number_timer_.reset() #restart the timer
        return super().on_activate(previous_state) #Call super on_activate
    
    #Disable HW
    def on_deactivate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_deactivate")
        self.number_timer_.cancel() #cancel the timer
        return super().on_deactivate(previous_state)
    
    #Destroy ROS2 communications, disconnect from HW 
    def on_cleanup(self, prevous_state: LifecycleState):
        self.get_logger().info("IN on_cleanup")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        return TransitionCallbackReturn.SUCCESS
    
    #Cleanup everything
    def on_shutdown(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_shutdown")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        return TransitionCallbackReturn.SUCCESS
    
    #Process errors, deactivate and cleanup
    def on_error(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_error")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        #Do some checks and if ok, then return SUCCESS, if not FAILURE
        return TransitionCallbackReturn.FAILSURE


    def publish_number(self):
        msg = Int64()
        msg.data = self.counter_
        self.number_publisher_.publish(msg)
        # self.get_logger().info(f"Publishing: {msg.data}")
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = NumberPUblisherNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()