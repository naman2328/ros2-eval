#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from custom_action.action import CruiseControl

class CruiseActionClientNode(Node):

    def __init__(self):
        super().__init__('cruise_client_node')

        # Create an ActionClient that connects to the cruise_speed action
        self.cruise_action_client = ActionClient(self, CruiseControl, "cruise_speed")
        
    def send_goal(self, cruise_speed, cruise_step):
        """ Send a goal to the action server """
        self.cruise_action_client.wait_for_server()

        goal = CruiseControl.Goal()
        goal.cruise_speed = cruise_speed  # Correct attribute name
        goal.cruise_step = cruise_step

        self.get_logger().info(f"Sending goal: {goal.cruise_speed}, Step: {goal.cruise_step}")

        # Send the goal asynchronously and set up feedback and result callbacks
        self.cruise_action_client.send_goal_async(goal, feedback_callback=self.feedback_callback).add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback):
        """ This callback is invoked every time the server sends feedback """
        current_speed = feedback.feedback.current_speed
        self.get_logger().info(f"Received feedback: Current speed is {current_speed} km/h")

    def goal_response_callback(self, future):
        """ This callback is invoked when the goal response is received (success/failure) """
        goal_handle = future.result()

        # Extract the result from the goal handle
        result = goal_handle.get_result()

        if result:
            self.get_logger().info(f"Goal reached successfully. Final speed is: {result.final_speed} km/h")
        else:
            self.get_logger().error("Goal failed.")

def main():
    rclpy.init()

    try:
        # Initialize the client node
        node = CruiseActionClientNode()
        
        # Send a goal to the action server (e.g., cruise speed: 50, step: 5)
        node.send_goal(50, 5)
        
        # Spin the node to process callbacks
        rclpy.spin(node)
        
        # Destroy the node and shut down once done
        node.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        print("Test interrupted by user. Exiting...")

if __name__ == '__main__':
    main()
