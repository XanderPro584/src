#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool

def main(args=None):
    rclpy.init(args=args)
    node = Node("arm_test")

    client = node.create_client(CommandBool, '/mavros/cmd/arming')

    while not client.wait_for_service(1):
        node.get_logger().warn("Waiting for Arm server")

    request = CommandBool.Request()
    request.value = True

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)



    try:
        response = future.result()
        node.get_logger().info(f"arming was {response.success}")
    except Exception as e:
        node.get_logger().error(f"Service call failed {e}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()