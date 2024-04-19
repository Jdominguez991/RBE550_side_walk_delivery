#!/usr/bin/env python
import rospy
import math
import tf
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

pose_gl = Odometry()

class MoveRobot():
    def __init__(self, velocity, rate):
        rospy.init_node('move_robot', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_tolerance_rotate = 0.01
        self.goal_tolerance_linear = 0.3
        self.position_tolerance_linear = 0.28
        self.pose = Odometry()
        self.rate = rospy.Rate(rate)  # Set the rate based on the argument
        self.velocity = velocity
        self.yaw = 0
        self.next_straight = False

    def next_point(self, goal_pose, next_point):
        x_goal = goal_pose.pose.position.x
        y_goal = goal_pose.pose.position.y
        
        if next_point:
            prev_x_goal = next_point.pose.position.x
            prev_y_goal = next_point.pose.position.y
            angle_to_next_goal = math.atan2(prev_y_goal - y_goal, prev_x_goal - x_goal)
                
            if(angle_to_next_goal < self.goal_tolerance_rotate and angle_to_next_goal > -(self.goal_tolerance_rotate)):
                self.next_straight = True
            else:
                self.next_straight = False

        print(self.next_straight)

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

            if(not self.next_straight):
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

                print("Angle riched!!!")
            else:
                print("Going straigth")
       
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def linear_movement(self, goal_pose, next_pose=None):
        try:
            
            x_goal = goal_pose.pose.position.x
            y_goal = goal_pose.pose.position.y
            
            x_act = self.pose.pose.pose.position.x
            y_act = self.pose.pose.pose.position.y

            x_diff = x_act - x_goal
            y_diff = y_act - y_goal
                
            distance_to_goal = math.sqrt((x_diff)**2 + (y_diff)**2)

            print("Driving to next spot...")
            while True:
                x_act = self.pose.pose.pose.position.x
                y_act = self.pose.pose.pose.position.y

                x_diff = x_act - x_goal
                y_diff = y_act - y_goal
                
                distance_to_goal = math.sqrt((x_diff)**2 + (y_diff)**2)

                # Set the linear velocity components
                
                if(not self.next_straight):
                    vel = self.velocity * distance_to_goal
                else:
                    vel = self.velocity
                    
                twist_msg = Twist()
                twist_msg.linear.x = vel
                twist_msg.linear.y = vel
                twist_msg.linear.z = 0 
                # Publish the twist message
                self.velocity_publisher.publish(twist_msg)
                
                # Check if the robot is close to the goal position
                if abs(distance_to_goal) < self.goal_tolerance_linear:
                    # Check if the robot's actual position is similar to the goal position
                    if (abs(x_diff) < self.position_tolerance_linear and 
                        abs(y_diff) < self.goal_tolerance_linear and
                        self.next_straight == False):
                        twist_msg = Twist()  # Stop the robot
                        self.velocity_publisher.publish(twist_msg)
                        print("Reached goal position...")
                        break
                    else:
                        twist_msg = Twist()
                        twist_msg.linear.x = self.velocity
                        twist_msg.linear.y = self.velocity
                        twist_msg.linear.z = 0
                        print("Next point straight...")
                        break

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
    
    def update_pose(self, data):
        self.pose = data


    def get_current_position(self):
        return self.pose.pose.pose.position.x, self.pose.pose.pose.position.y, self.yaw
    
    def navigate_to_next_goal(self, point, next_goal_point=None):
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = point[0]
        goal_pose.pose.position.y = point[1]

        self.next_point(goal_pose, next_goal_point)
        self.calculate_angle(goal_pose)
        self.linear_movement(goal_pose, next_goal_point)


def update_pose_callback(data, move_robot_instance):
    move_robot_instance.update_pose(data)
    global pose_gl 
    pose_gl = data


if __name__ == '__main__':
    try:

        points = [[2.5, 0.0],[7.0, 0.0],[8,0.5]]
        velocity = 0.8
        rate = 10

        move_robot = MoveRobot(velocity, rate)  # Pass the rate to the constructor
        move_robot.pose_subscriber = rospy.Subscriber('/odom', Odometry, lambda data: update_pose_callback(data, move_robot))
        print(pose_gl.pose.pose.orientation.y)
        # Initialize previous goal pose as None
        next_goal_point = None

        for i in range(len(points)):
            point = points[i]
            next_point = points[i + 1] if i + 1 < len(points) else None
            if(next_point):
            # Update the previous goal pose
                next_goal_point = PoseStamped()
                next_goal_point.pose.position.x = point[0]
                next_goal_point.pose.position.y = point[1]
            # Get the current position
            current_x, current_y, current_orientation = move_robot.get_current_position()
            print("Current position:", current_x, current_y, current_orientation)

            # Navigate to the next goal, passing the previous goal pose
            move_robot.navigate_to_next_goal(point, next_goal_point)       
        
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
