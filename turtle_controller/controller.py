#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
from my_robot_interfaces.srv import ToggleTurtleState
from turtlesim.srv import SetPen

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
                myMsg.angular.z = np.random.randn() * 0.5   
                self.cmdVel_pub.publish(myMsg)
            if self.isReachingWall(currentPose) and not self.isReachingWall(self.prevPose):
                self.get_logger().info('changing pen color to red')
                self.setPenService_call(255, 0, 0, 2, 0)
            if self.isReachingWall(self.prevPose) and not self.isReachingWall(currentPose):
                self.get_logger().info('changing pen color to blue')
                self.setPenService_call(0, 255, 0, 1, 0)
            self.prevPose = currentPose

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