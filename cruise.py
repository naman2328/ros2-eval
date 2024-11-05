#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from custom_action.action import CruiseControl
from std_msgs.msg import Int64
import time

class CruiseActionServerNode(Node):

    def __init__(self):
        super().__init__('cruise_node')

        # Declare the 'speed' parameter, with a default value of 80
        self.declare_parameter('speed', 80)  # Default value 80, can be overridden by launch file

        # Get the value of the 'speed' parameter
        self.speed = self.get_parameter('speed').get_parameter_value().integer_value

        # Initialize the action server
        self.cruise_action_server = ActionServer(self, CruiseControl, "cruise_speed", execute_callback=self.callback_cruise_server)
        
        # Initialize the publisher for final speed
        self.final_speed_publisher = self.create_publisher(Int64, 'final_speed_topic', 10)

        self.get_logger().info("Action Server Node Started !! Waiting For request")
        self.get_logger().info(f"Initial speed set to {self.speed}")

    def callback_cruise_server(self, handle):
        cruise_speed = handle.request.cruise_speed
        cruise_step = handle.request.cruise_step
        current_speed = self.speed

        self.get_logger().info(f"Requested cruise speed: {cruise_speed}, Step size: {cruise_step}")

        # Check if we need to increase or decrease speed
        if cruise_speed > current_speed:
            # Increase speed (add steps)
            while current_speed < cruise_speed:
                if current_speed + cruise_step > cruise_speed:
                    current_speed = cruise_speed  # Stop at cruise_speed if we would overshoot
                    break
                current_speed += cruise_step
                self.get_logger().info(f"Increasing speed: {current_speed}")
                time.sleep(1)

        elif cruise_speed < current_speed:
            # Decrease speed (subtract steps)
            while current_speed > cruise_speed:
                if current_speed - cruise_step < cruise_speed:
                    current_speed = cruise_speed  # Stop at cruise_speed if we would undershoot
                    break
                current_speed -= cruise_step
                self.get_logger().info(f"Decreasing speed: {current_speed}")
                time.sleep(1)

        # Log message when the goal is reached
        self.get_logger().info(f"Goal reached. Final speed is: {current_speed}")

        # Publish the final speed after achieving the target speed
        final_speed_msg = Int64()
        final_speed_msg.data = current_speed
        self.final_speed_publisher.publish(final_speed_msg)

        # Mark the action as succeeded
        handle.succeed()

        # Prepare and return the result
        result = CruiseControl.Result()
        result.final_speed = current_speed
        return result

def main():
    rclpy.init()

    try:
        node = CruiseActionServerNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        print("Test interrupted by user. Exiting...")

if __name__ == '__main__':
    main()
