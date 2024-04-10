#!/usr/bin/env python
import rospy
import math
import tf
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler


class MoveRobot():
    def __init__(self, velocity):
        rospy.init_node('move_robot', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.update_pose)
        self.goal_tolerance_rotate = 0.01
        self.goal_tolerance_linear = 0.3
        self.position_tolerance_linear = 0.28
        self.pose = Odometry()
        self.rate = rospy.Rate(10)
        self.velocity = velocity

    def update_pose(self, data):
        self.pose = data

    def calculate_angle(self, goal_pose):
        try:
            print("Rotating to correct angle...")
            x_act = self.pose.pose.pose.position.x
            y_act = self.pose.pose.pose.position.y
            z_act = self.pose.pose.pose.orientation
            
            _, _, yaw = euler_from_quaternion([z_act.x, z_act.y, z_act.z, z_act.w])

            x = goal_pose.pose.position.x
            y = goal_pose.pose.position.y
            
    
            angle_to_next_goal = math.atan2(y - y_act, x - x_act)
            print(x_act,x,y_act,y,angle_to_next_goal-yaw)

            while (abs(angle_to_next_goal - yaw) > self.goal_tolerance_rotate):
                z_act = self.pose.pose.pose.orientation
                _, _, yaw = euler_from_quaternion([z_act.x, z_act.y, z_act.z, z_act.w])

                twist_msg = Twist()
                # Calculate angular velocity to rotate towards the next goal point
                angular_velocity = (angle_to_next_goal - yaw) * self.velocity

                # Create Twist message to rotate the robot
                twist_msg.angular.z = angular_velocity
                self.velocity_publisher.publish(twist_msg)

            twist_msg = Twist()
            self.velocity_publisher.publish(twist_msg)  # Stop the robot
            print("Angle riched!")
            self.rate.sleep()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def linear_movement(self, goal_pose):
        try:
            print("Driving to next spot...")
            x_goal = goal_pose.pose.position.x
            y_goal = goal_pose.pose.position.y

            while True:
                x_act = self.pose.pose.pose.position.x
                y_act = self.pose.pose.pose.position.y

                distance_to_goal = math.sqrt((x_act - x_goal)**2 + (y_act - y_goal)**2)

                # Set the linear velocity components
                twist_msg = Twist()
                twist_msg.linear.x = self.velocity
                twist_msg.linear.y = self.velocity
                twist_msg.linear.z = 0

                # Publish the twist message
                self.velocity_publisher.publish(twist_msg)

                # Check if the robot is close to the goal position
                if distance_to_goal < self.goal_tolerance_linear:
                    # Check if the robot's actual position is similar to the goal position
                    if abs(x_act - x_goal) < self.position_tolerance_linear and abs(y_act - y_goal) < self.goal_tolerance_linear:
                        twist_msg = Twist()  # Stop the robot
                        self.velocity_publisher.publish(twist_msg)
                        print("Reached goal position")
                        break

                self.rate.sleep()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
 
    def navigate_to_next_goal(self, point,num):
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = point[0]
        goal_pose.pose.position.y = point[1]
        #goal_pose.pose.orientation.z = point[2]

        self.calculate_angle(goal_pose)
        self.linear_movement(goal_pose)
        self.rate.sleep()

if __name__ == '__main__':
    try:
        points = [[1.5, 0.0],[1.7, 1.0],[2.3,-0.7]]
        velocity = 0.8

        move_robot = MoveRobot(velocity)
        num = 0
        for point in points:
            move_robot.navigate_to_next_goal(point,num)
            num =+ 1
        
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
