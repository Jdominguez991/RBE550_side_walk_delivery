#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt

class MoveRobot():
    def __init__(self):
        rospy.init_node('move_robot', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.update_pose)

        self.pose = Odometry()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        self.pose = data

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.pose.position.x - self.pose.pose.pose.position.x), 2) +
                    pow((goal_pose.pose.position.y - self.pose.pose.pose.position.y), 2))

    def linear_vel(self, goal_pose, constant=0.5):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.pose.position.y - self.pose.pose.pose.position.y,
                     goal_pose.pose.position.x - self.pose.pose.pose.position.x)

    def angular_vel(self, goal_pose, constant=1):
        return constant * (self.steering_angle(goal_pose) - self.pose.pose.pose.orientation.z)

    def move2goal(self, goal_pose, velocity):
        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= 0.01:
            # Linear velocity
            vel_msg.linear.x = min(self.linear_vel(goal_pose), velocity)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # Stopping the robot after reaching the goal
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        x_goal, y_goal, orientation = [1.2, 0.2, 0]
        velocity = 0.5

        move_robot = MoveRobot()

        # Define goal pose
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = x_goal
        goal_pose.pose.position.y = y_goal
        goal_pose.pose.orientation.z = orientation

        move_robot.move2goal(goal_pose, velocity)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
