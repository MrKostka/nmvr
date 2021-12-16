import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

class RobotOdometry(Node):
    def __init__(self):
        super().__init__("robot_odometry")
        self.goal_pub = self.create_publisher(Pose, 'pose', 10)
        self.velocity_subs = self.create_subscription(Twist,'velocity',self.velocity_callback,10)

        self.timer_period = 1/60  # seconds
        self.timer = self.create_timer(self.timer_period, self.pose_callback)

        self._pose = Pose()

        self.velocity = Twist()

    def pose_callback(self):

        next_pose = Pose()

        next_pose.theta = self._pose.theta + (self.velocity.angular.z * self.timer_period)

        next_pose.x = self._pose.x + (self.velocity.linear.x * self.timer_period) * math.cos(next_pose.theta)
        next_pose.y = self._pose.y + (self.velocity.linear.x * self.timer_period) * math.sin(next_pose.theta)

        self._pose = next_pose

        self.goal_pub.publish(self._pose)   

    def velocity_callback(self,vel_msg):

        self.velocity.linear.x = vel_msg.linear.x
        self.velocity.linear.y = vel_msg.linear.y
        self.velocity.angular.z = vel_msg.angular.z

def main(args=None):
    rclpy.init(args=args)
    robot_odometry = RobotOdometry()
    rclpy.spin(robot_odometry)    
    rclpy.shutdown()
    robot_odometry.destroy_node()

if __name__ == '__main__':
    main()