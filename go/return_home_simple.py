#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
import tf2_ros

class ReturnHomeNav2(Node):
    """
    Node that coordinates the return journey by navigating the exact reverse
    of the recorded exploration path using Nav2.
    """
    def __init__(self):
        super().__init__('return_home_nav2')
        
        # Set up service to trigger return
        self.return_service = self.create_service(
            Trigger, 
            '/return_home', 
            self.return_home_callback
        )
        
        # Subscribe to the exploration path
        self.explore_path_sub = self.create_subscription(
            Path,
            '/path_explore',
            self.explore_path_callback,
            10
        )
        self.exploration_path = None
        
        # Set up publisher for return path visualization
        self.return_path_publisher = self.create_publisher(
            Path,
            '/path_return',
            10
        )
        
        # Set up action client for Nav2
        self.nav_client = ActionClient(
            self, 
            NavigateToPose, 
            '/navigate_to_pose'
        )
        
        # Set up TF listener for tracking the actual return path
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Path recording variables
        self.return_path = Path()
        self.return_path.header.frame_id = 'map'
        self.return_poses = []
        self.last_return_position = None
        
        # Status tracking variables
        self.is_returning = False
        self.current_waypoint_index = 0
        self.waypoints = []
        
        # Timer for tracking return position (1 Hz)
        self.return_tracker_timer = self.create_timer(1.0, self.track_return_position)
        
        self.get_logger().info('Return home node initialized')
        self.get_logger().info('Service "/return_home" is now available')
        self.get_logger().info('Waiting for exploration path...')
    
    def explore_path_callback(self, msg):
        """
        Callback for receiving the exploration path.
        Only process updates when not in return mode.
        """
        # Skip updates if we're already returning
        if self.is_returning:
            return
            
        self.exploration_path = msg
        path_length = len(self.exploration_path.poses)
        self.get_logger().info(f'Received exploration path with {path_length} points')
    
    def return_home_callback(self, request, response):
        """
        Service callback to trigger return to home.
        """
        self.get_logger().info('Return home service called')
        
        if self.is_returning:
            self.get_logger().warn('Already returning home')
            response.success = False
            response.message = "Already returning home"
            return response
        
        if self.exploration_path is None or len(self.exploration_path.poses) < 1:
            self.get_logger().warn('No exploration path available')
            response.success = False
            response.message = "No exploration path available"
            return response
        
        # Take a snapshot of the current exploration path
        self.get_logger().info('Taking snapshot of exploration path')
        current_path_length = len(self.exploration_path.poses)
        
        # Plan the return path (simply reverse the exploration path)
        self.get_logger().info('Planning return path based on snapshot')
        self.waypoints = list(reversed(self.exploration_path.poses))
        
        # Start the return journey
        self.is_returning = True
        self.current_waypoint_index = 0
        self.return_poses = []  # Clear return poses
        self.return_path.poses = []
        self.last_return_position = None
        
        # Start navigating to the first waypoint
        self.navigate_to_next_waypoint()
        
        response.success = True
        response.message = f"Starting return journey with {current_path_length} waypoints"
        return response
    
    def navigate_to_next_waypoint(self):
        """
        Send navigation goal for the next waypoint.
        """
        if not self.is_returning:
            return
        
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("Return journey completed!")
            self.is_returning = False
            return
        
        # Get the next waypoint
        current_waypoint = self.waypoints[self.current_waypoint_index]
        
        # Create the navigation goal
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose = current_waypoint
        
        self.get_logger().info(f'Navigating to waypoint {self.current_waypoint_index+1}/{len(self.waypoints)}')
        
        # Wait for the action server
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation action server not available')
            self.is_returning = False
            return
        
        # Send the goal
        self._send_goal_future = self.nav_client.send_goal_async(
            goal_pose,
            feedback_callback=self.nav_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.nav_goal_response_callback)
    
    def nav_goal_response_callback(self, future):
        """
        Callback for handling the response to a navigation goal request.
        """
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal was rejected')
            # Try the next waypoint
            self.current_waypoint_index += 1
            self.navigate_to_next_waypoint()
            return
        
        self.get_logger().info('Navigation goal accepted')
        
        # Get the result future
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.nav_result_callback)
    
    def nav_result_callback(self, future):
        """
        Callback for handling the result of a navigation goal.
        """
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Successfully reached waypoint {self.current_waypoint_index+1}')
            # Move to the next waypoint
            self.current_waypoint_index += 1
            self.navigate_to_next_waypoint()
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')
            # Try the next waypoint
            self.current_waypoint_index += 1
            self.navigate_to_next_waypoint()
    
    def nav_feedback_callback(self, feedback_msg):
        """
        Callback for handling navigation feedback.
        """
        # We use TF for tracking rather than feedback
        pass
    
    def track_return_position(self):
        """
        Track the robot's position during return and add it to the return path.
        """
        if not self.is_returning:
            return
            
        try:
            # Get the latest transform
            transform = self.tf_buffer.lookup_transform(
                'map',               # Target frame
                'base_link',         # Source frame
                rclpy.time.Time(),   # Latest available transform
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Check if position has changed enough to record
            record_position = False
            if self.last_return_position is None:
                # First position
                record_position = True
            else:
                # Calculate distance from last position
                dx = x - self.last_return_position[0]
                dy = y - self.last_return_position[1]
                distance = (dx*dx + dy*dy)**0.5
                
                # Record if moved at least 0.1 meters
                if distance >= 0.1:
                    record_position = True
            
            if record_position:
                # Create pose message
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = transform.header.stamp
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = transform.transform.translation.z
                pose.pose.orientation = transform.transform.rotation
                
                # Add to return path
                self.return_poses.append(pose)
                self.last_return_position = (x, y)
                
                # Update and publish return path
                self.return_path.header.stamp = transform.header.stamp
                self.return_path.poses = self.return_poses
                self.return_path_publisher.publish(self.return_path)
                
        except tf2_ros.TransformException as ex:
            self.get_logger().warning(f'Could not transform: {ex}')

def main(args=None):
    rclpy.init(args=args)
    
    return_home_node = ReturnHomeNav2()
    
    try:
        rclpy.spin(return_home_node)
    except KeyboardInterrupt:
        pass
    
    return_home_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()