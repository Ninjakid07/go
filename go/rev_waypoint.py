#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String


class ReverseWaypointFollower(Node):
    """
    Node that follows a series of waypoints in reverse order.
    Subscribes to a path and navigates through its waypoints in reverse.
    """
    def __init__(self):
        super().__init__('reverse_waypoint_follower')
        
        # Initialize waypoints list and navigation status
        self.waypoints = []
        self.current_waypoint_index = 0
        self.is_navigating = False
        
        # Create path subscriber
        self.path_sub = self.create_subscription(
            Path,
            '/recorded_path',
            self.path_callback,
            10
        )
        
        # Create Nav2 action client
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # Create status publisher
        self.status_publisher = self.create_publisher(
            String, 
            '/snc_status', 
            10
        )
        
        self.get_logger().info('Reverse waypoint follower initialized')
        self.get_logger().info('Waiting for path on /recorded_path')
        
        # Publish initial status
        self.publish_status("Initialized - Waiting for path")
    
    def path_callback(self, msg):
        """
        Callback for receiving the path.
        Reverses the path and starts navigation if not already navigating.
        """
        if self.is_navigating:
            self.get_logger().info('Already navigating - ignoring new path')
            return
        
        # Extract and reverse waypoints
        self.waypoints = list(reversed(msg.poses))
        num_waypoints = len(self.waypoints)
        
        if num_waypoints == 0:
            self.get_logger().warn('Received empty path - nothing to navigate')
            self.publish_status("Received empty path - nothing to navigate")
            return
        
        self.get_logger().info(f'Received path with {num_waypoints} waypoints (reversed for return journey)')
        self.publish_status(f"Starting return journey with {num_waypoints} waypoints")
        
        # Reset navigation state
        self.current_waypoint_index = 0
        self.is_navigating = True
        
        # Start navigation to first waypoint
        self.navigate_to_next_waypoint()
    
    def navigate_to_next_waypoint(self):
        """
        Navigate to the next waypoint in the sequence.
        """
        if not self.is_navigating:
            return
        
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("Navigation complete! Reached all waypoints.")
            self.publish_status("Navigation complete! Reached all waypoints.")
            self.is_navigating = False
            return
        
        # Get current waypoint
        current_waypoint = self.waypoints[self.current_waypoint_index]
        
        # Create navigation goal
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose = current_waypoint
        
        self.get_logger().info(f'Navigating to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}')
        self.publish_status(f"Navigating to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}")
        
        # Wait for the action server
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation action server not available')
            self.publish_status("Error: Navigation server not available")
            self.is_navigating = False
            return
        
        # Send the goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_pose,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """
        Callback for handling the response to a navigation goal request.
        """
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal was rejected')
            self.publish_status(f"Waypoint {self.current_waypoint_index + 1} rejected")
            
            # Move to the next waypoint
            self.current_waypoint_index += 1
            self.navigate_to_next_waypoint()
            return
        
        self.get_logger().info('Navigation goal accepted')
        
        # Get the result future
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """
        Callback for handling the result of a navigation goal.
        """
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Successfully reached waypoint {self.current_waypoint_index + 1}')
            self.publish_status(f"Reached waypoint {self.current_waypoint_index + 1}")
        else:
            self.get_logger().warn(f'Navigation to waypoint {self.current_waypoint_index + 1} failed with status: {status}')
            self.publish_status(f"Failed waypoint {self.current_waypoint_index + 1}")
        
        # Move to the next waypoint regardless of success or failure
        self.current_waypoint_index += 1
        self.navigate_to_next_waypoint()
    
    def feedback_callback(self, feedback_msg):
        """
        Callback for handling navigation feedback.
        """
        # We could process feedback here if needed
        pass
    
    def publish_status(self, status_text):
        """
        Publish status message.
        """
        msg = String()
        msg.data = status_text
        self.status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = ReverseWaypointFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()