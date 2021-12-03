import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from turtlesim.msg import Pose

import sys

import math
from math import pow, atan2, sqrt

import logging

class Robot(Node):
    def __init__(self):
        super().__init__("robot")

        self.velocity_pub = self.create_publisher(Twist,'velocity',10)

        self.pose_subs = self.create_subscription(Pose,'pose',self.pose_callback,20)
        self.goal_subs = self.create_subscription(Pose,'goal',self.goal_callback,20)
        
        timer_period = 0.5
        self.timer = self.create_timer(timer_period,self.p_control)

        self.pose = Pose()
        self.goal_pose = Pose()
        self.flag = False

    def pose_callback(self,data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose.x = data.x
        self.pose.y = data.y
        self.pose.theta = data.theta
        # self.get_logger().info("Robot pose callback")
        # msg = 'X: {:.3f}, Y: {:.3f}, Theta: {:.3f}'.format(data.x, data.y, data.theta)
        # self.get_logger().info(msg)

    def goal_callback(self,data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.goal_pose.x = data.x
        self.goal_pose.y = data.y
        self.goal_pose.theta = data.theta
        # self.get_logger().info("Robot goal callback")
        # msg = 'X: {:.3f}, Y: {:.3f}, Theta: {:.3f}'.format(data.x, data.y, data.theta)
        # self.get_logger().info(msg)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=2):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        angle = atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
        return angle

    def angular_vel(self, goal_pose, constant=2):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)
        

    def p_control(self,):
        """Moves the turtle to the goal."""
        distance_tolerance = 0.1
        angular_tolerance = 0.1
        self.flag = False

        vel_msg = Twist()

        if abs(self.steering_angle(self.goal_pose) - self.pose.theta) > angular_tolerance:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = self.angular_vel(self.goal_pose)  
            if vel_msg.angular.z > (2 * math.pi):
                vel_msg.angular.z = (2 * math.pi) - 0.25  

            if vel_msg.angular.z < -(2 * math.pi):
                vel_msg.angular.z = (-1 * (2 * math.pi)) + 0.25
        else:
            vel_msg.angular.z = 0.0
            if self.euclidean_distance(self.goal_pose) >= distance_tolerance:                
                vel_msg.linear.x = self.linear_vel(self.goal_pose)
                if vel_msg.linear.x > 40.0:
                    vel_msg.linear.x = 40.0
                
                if vel_msg.linear.x < -40.0:
                    vel_msg.linear.x = -40.0
            else:                
                vel_msg.linear.x = 0.0
                self.flag = True
        
        if self.flag:
            vel_msg.angular.z = self.goal_pose.theta - self.pose.theta
            if abs(self.goal_pose.theta-self.pose.theta) <= angular_tolerance:
                # quit()
                vel_msg.linear.x = 0.0
                vel_msg.linear.y = 0.0
                vel_msg.angular.z = 0.0
        
        self.velocity_pub.publish(vel_msg)

        self.get_logger().info('vypis z konca p regulatora:')
        msg = 'pose: \t X: {:.3f}, Y: {:.3f}, Theta: {:.3f}'.format(self.pose.x, self.pose.y, self.pose.theta)
        self.get_logger().info(msg)
        msg = 'vel_msg: \t .linear.x: {:.3f}, .linear.y: {:.3f}, .angular.z: {:.3f}'.format(vel_msg.linear.x, vel_msg.linear.y, vel_msg.angular.z)
        self.get_logger().info(msg)
        msg = 'goal pose: \t X: {:.3f}, Y: {:.3f}, Theta: {:.3f}'.format(self.goal_pose.x, self.goal_pose.y, self.goal_pose.theta)
        self.get_logger().info(msg)

def main(aargs=None):
    rclpy.init()
    node = Robot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()