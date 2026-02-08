#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
# Sub/Pub example imports
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
# service example imports
from my_robot_interfaces.srv import ToggleTurtleState
from turtlesim.srv import SetPen
# parameter example imports
from .ros_param_fns import MOTION_GENERATORS, gaussian_motion, PEN_COLORS
# action example imports
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from my_robot_interfaces.action import TurnTurtle


uint8_ = lambda x: max(0, min(255, x)) 

class MyTurtleControllerNode(Node): 
    def __init__(self):
        super().__init__("turtle_controller") 
        self.get_logger().info("Turtle Controller has started.")
        
        # Publisher and Subscriber
        self.cmdVel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.poseSubscriber_cb, 10)
        
        # Service Server and Client
        self.turtle_toggle_srv = self.create_service(
            ToggleTurtleState, 
            "turtle_state_toggle_service", 
            self.turtleToggleService_cb
        )
        self.mySetPenClient = self.create_client(SetPen, '/turtle1/set_pen')
        
        # State variables
        self.prevPose = Pose()
        self.current_pose = Pose()  # Initialize current_pose
        self.is_active = True
        
        # Parameters
        self.declare_parameter("motion_pattern", "Cauchy")
        self.declare_parameter("motion_std", 0.5)
        self.declare_parameter("primary_color", "Blue")
        self.declare_parameter("secondary_color", "Red")
        
        # Action server state
        self.action_active = False
        self.wall_manouver = False
        self.action_cmdVel = Twist()
        self.normal_cmdVel = Twist()  # Initialize normal command velocity
        
        # Action server
        self.turn_turtle_srv = ActionServer(
            self, 
            TurnTurtle, 
            "turn_turtle_srv",
            goal_callback=self.goal_cb,
            execute_callback=self.execute_cb,  # Fixed typo: was execution_cb
            cancel_callback=self.cancel_cb,
            callback_group=ReentrantCallbackGroup()
        )
        
        # Control loop timer - single publishing point
        self.control_timer = self.create_timer(0.1, self.control_loop)

    def poseSubscriber_cb(self, pose):
        """Update current pose"""
        self.current_pose = pose

    def control_loop(self):
        ''' Main control loop - centralized velocity publishing '''
        if not self.is_active:
            # Turtle is deactivated, stop it
            stop_msg = Twist()
            self.cmdVel_pub.publish(stop_msg)
            return
        
        # Determine which command to publish
        if self.action_active:
            # Action mode - publish action command
            self.cmdVel_pub.publish(self.action_cmdVel)
        else:
            # Normal mode - compute and publish normal motion
            my_cmdVel_msg = Twist()
            
            if self.isReachingWall(self.current_pose):
                self.get_logger().info('hitting wall; turning around')
                my_cmdVel_msg.linear.x = 1.0
                my_cmdVel_msg.angular.z = 1.5
            else:
                my_cmdVel_msg.linear.x = np.random.normal(1.0, 0.3)
                
                # Get motion generator based on parameter
                generator = MOTION_GENERATORS.get(
                    self.get_parameter("motion_pattern").value, 
                    gaussian_motion
                )
                my_cmdVel_msg.angular.z = generator(
                    self.current_pose.angular_velocity,
                    self.get_parameter("motion_std").value
                )
            
            # Publish normal command
            self.cmdVel_pub.publish(my_cmdVel_msg)
            
            # Handle pen color changes (only in normal mode)
            if self.isReachingWall(self.current_pose) and not self.isReachingWall(self.prevPose):
                color = self.get_parameter("primary_color").value
                self.get_logger().info('changing pen color to ' + color)
                self.setPenService_call(*PEN_COLORS.get(color, (128, 0, 128)), 2, 0)
            
            if self.isReachingWall(self.prevPose) and not self.isReachingWall(self.current_pose):
                color = self.get_parameter("secondary_color").value
                self.get_logger().info('changing pen color to ' + color)
                self.setPenService_call(*PEN_COLORS.get(color, (0, 255, 255)), 1, 0)
        
        # Update previous pose (outside the if/else)
        self.prevPose = self.current_pose

    def isReachingWall(self, pose):
        """Check if turtle is near arena boundaries"""
        if pose.x > 10 or pose.x < 1 or pose.y > 10 or pose.y < 1:
            return True
        else:
            return False
        
    def turtleToggleService_cb(self, request: ToggleTurtleState.Request, response: ToggleTurtleState.Response):
        """Handle toggle service requests"""
        assert isinstance(request.turtle_switch, bool), 'Toggle Must be boolean.'
        self.is_active = request.turtle_switch
        if request.turtle_switch:
            response.success = True
            response.turtle_status = 'Turtle is activated'
        else:
            response.success = True
            response.turtle_status = 'Turtle is deactivated'
        return response
    
    def setPenService_call(self, r: int, g: int, b: int, width: int, penOff: int):
        """Call SetPen service to change pen properties"""
        self.get_logger().info('Converting values to Uint8:' + str([r, g, b, width, penOff]))
        while not self.mySetPenClient.wait_for_service(1.0):
            self.get_logger().info('Waiting for service.')
        request = SetPen.Request()
        request.r = uint8_(r)
        request.g = uint8_(g)
        request.b = uint8_(b)
        request.width = uint8_(width)
        request.off = uint8_(penOff)
        future = self.mySetPenClient.call_async(request)
        future.add_done_callback(self.setPenClient_cb)
    
    def setPenClient_cb(self, future):
        """Handle SetPen service response"""
        response = future.result()
        self.get_logger().info("Server Response: " + str(response))

    # ============================================
    # ACTION SERVER CALLBACKS
    # ============================================

    def goal_cb(self, goal_request):
        """Goal callback - decide whether to accept or reject the goal"""
        self.get_logger().info('Goal Received!')
        
        # Check if turtle is active
        if not self.is_active:
            self.get_logger().info('Goal rejected: Turtle is deactivated')
            return GoalResponse.REJECT
        
        # Check if turtle is near wall
        if self.wall_manouver or self.isReachingWall(self.current_pose):
            self.get_logger().info('Goal rejected: Turtle is near wall')
            return GoalResponse.REJECT
        
        # Check if action is already in progress
        if self.action_active:
            self.get_logger().info('Goal rejected: Action already in progress')
            return GoalResponse.REJECT
        
        self.get_logger().info('Accepting Goal!')
        return GoalResponse.ACCEPT

    def execute_cb(self, goal_handle: ServerGoalHandle):
        """Execute callback - main action logic"""
        self.get_logger().info("Executing the goal")
        self.action_active = True
        
        # Get the turn degree from goal
        deg_to_turn = goal_handle.request.turn_degree
        turn_radians = np.radians(deg_to_turn)
        
        # Calculate duration based on angular velocity
        angular_velocity = 0.3  # rad/s ; defines the rate at which the Turtle Turns
        duration = abs(turn_radians / angular_velocity)
        
        # Set action command
        self.action_cmdVel.linear.x = 0.0
        self.action_cmdVel.angular.z = angular_velocity if deg_to_turn > 0 else -angular_velocity
        
        # Create feedback
        feedback = TurnTurtle.Feedback()
        
        # Execute turn with feedback
        start_time = self.get_clock().now()
        start_angle = self.current_pose.theta
        rate = self.create_rate(10)  # 10 Hz rate of fb, progress check, and cancel request check
        
        while True:
            # Check if goal was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.action_active = False
                self.get_logger().info('Goal canceled')
                
                # Stop the turtle by resetting action command
                self.action_cmdVel = Twist()    # empty cmdVel
                result = TurnTurtle.Result()
                result.heading = self.current_pose.theta
                return result
            
            # Calculate elapsed time
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            
            if elapsed >= duration:
                # Turn complete
                break
            
            # Calculate progress
            angle_turned = abs(self.current_pose.theta - start_angle)
            remaining = abs(turn_radians) - angle_turned
            progress = (elapsed / duration) * 100.0
            
            # Publish feedback
            feedback.current_angle = np.degrees(self.current_pose.theta)
            feedback.remaining_angle = np.degrees(remaining)
            feedback.progress = progress
            goal_handle.publish_feedback(feedback)
            
            self.get_logger().info(
                f'Progress: {progress:.1f}%, Remaining: {np.degrees(remaining):.1f} deg'
            )
            
            rate.sleep()
    
        # Stop the turtle by resetting action command
        self.action_cmdVel = Twist()
        
        # Action succeeded
        self.action_active = False
        goal_handle.succeed()
        
        result = TurnTurtle.Result()
        result.heading = self.current_pose.theta
        
        self.get_logger().info(f'Goal succeeded! Final heading: {np.degrees(result.heading):.2f} deg')
        return result
    
    def cancel_cb(self, goal_handle: ServerGoalHandle):
        """Cancel callback - decide whether to accept cancellation"""
        self.get_logger().info('Cancel Request Received')
        return CancelResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)
    node = MyTurtleControllerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()