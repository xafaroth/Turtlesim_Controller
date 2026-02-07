#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
from my_robot_interfaces.srv import ToggleTurtleState
from turtlesim.srv import SetPen
from .ros_param_fns import MOTION_GENERATORS, gaussian_motion, PEN_COLORS

uint8_ = lambda x: max(0, min(255, x)) 

class MyTurtleControllerNode(Node): 
    def __init__(self):
        super().__init__("turtle_controller") 
        self.cmdVel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)  # publisher to change velocity of turtle
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.poseSubscriber_cb, 10)    # subscriber to get turtle inertial info
        self.turtle_toggle_srv = self.create_service(ToggleTurtleState, 
                                                     "turtle_state_toggle_service", 
                                                     self.turtleToggleService_cb) # custom service to toggle turtle state (on/off) 
        self.get_logger().info("Turtle Controller has started.")
        self.mySetPenClient = self.create_client(SetPen, '/turtle1/set_pen')    # client to change pen properties
        self.prevPose = Pose()
        self.is_active = True
        # parameters
        self.declare_parameter("motion_pattern", "Gaussian")
        self.declare_parameter("motion_std", 0.5)
        self.declare_parameter("primary_color", "Blue")
        self.declare_parameter("secondary_color", "Red")

    def poseSubscriber_cb(self, currentPose):
        if self.is_active:
            myMsg = Twist()
            if self.isReachingWall(currentPose):
                self.get_logger().info('hitting wall; turning around')
                myMsg.linear.x = 1.0
                myMsg.angular.z = 1.5
                self.cmdVel_pub.publish(myMsg)
            else:
                myMsg.linear.x = np.random.normal(1.0, 0.3)  # Mean=1.0, StdDev=0.3
                myMsg.linear.y = np.random.normal(1.0, 0.3)  # Mean=1.0, StdDev=0.3
                generator = MOTION_GENERATORS.get(self.get_parameter("motion_pattern").value, gaussian_motion)
                myMsg.angular.z = generator(currentPose.angular_velocity, self.get_parameter("motion_std").value)
                self.cmdVel_pub.publish(myMsg)
            if self.isReachingWall(currentPose) and not self.isReachingWall(self.prevPose):
                color = self.get_parameter("primary_color").value
                self.get_logger().info('changing pen color to '+color)
                self.setPenService_call(*PEN_COLORS.get(color, (128, 0, 128)), 2, 0)
            if self.isReachingWall(self.prevPose) and not self.isReachingWall(currentPose):
                color = self.get_parameter("secondary_color").value
                self.get_logger().info('changing pen color to '+color)
                self.setPenService_call(*PEN_COLORS.get(color, (0, 255, 255)), 1, 0)
            self.prevPose = currentPose

    # def get_angularZ_motion(self, angular_z):
    #     dist = self.get_parameter("motion_pattern").value
    #     std = self.get_parameter("motion_std").value
    #     if dist is "Uniform":
    #         return np.random.uniform(-3.0, 3.0)
    #     elif dist is "Cauchy":
    #         return np.random.standard_cauchy() * std
    #     elif dist is "Poission":
    #         np.random.choice([-1.5, -.5, 0, .5, 1.5]) * std
    #     elif dist is "Exponential":
    #         np.random.exponential(std) * np.random.choice(-1, 1)
    #     elif dist is "Brownian":
    #         angular_z += np.random.randn() * std
    #         return np.clip(angular_z, -3.0, 3.0)
    #     else:
    #         return np.random.randn() * std        

    def isReachingWall(self, pose):
        # if np.abs(poseMsg.x - 5.5)>=5.0 or np.abs(poseMsg.y - 5.5)>=5.0:
        if pose.x > 10 or pose.x < 1 or pose.y > 10 or pose.y < 1:
            return True
        else:
            return False
        
    def turtleToggleService_cb(self, request:ToggleTurtleState.Request, response:ToggleTurtleState.Response):
        assert isinstance(request.turtle_switch, bool), 'Toggle Must be boolean.'
        self.is_active = request.turtle_switch
        if request.turtle_switch:
            response.success = True
            response.turtle_status = 'Turtle is activated'
        else:
            response.success = True
            response.turtle_status = 'Turtle is deactivated'
        return response
    
    def setPenService_call(self, r: int, g: int, b:int, width:int, penOff:int):
        self.get_logger().info('Converting values to Uint8:'+str([r,g,b,width, penOff]))            
        while not self.mySetPenClient.wait_for_service(1.0):
            self.get_logger().info('Waiting for service.')
        request = SetPen.Request()
        request.r, request.g, request.b, request.width, request.off = uint8_(r), uint8_(g), uint8_(b), uint8_(width), uint8_(penOff)
        future = self.mySetPenClient.call_async(request)
        future.add_done_callback(self.setPenClient_cb)
    
    def setPenClient_cb(self, future):
        response = future.result()
        self.get_logger().info("Server Response: " + str(response))


def main(args=None):
    rclpy.init(args=args)
    node = MyTurtleControllerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()