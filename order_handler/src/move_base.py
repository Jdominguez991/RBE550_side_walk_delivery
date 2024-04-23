#!/usr/bin/env python
import rospy
import math
import tf
from geometry_msgs.msg import Twist, PoseStamped, Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from motion_controller.srv import path, pathRequest

import sys
pose_gl = Odometry()

class MoveRobot():
    def __init__(self, velocity, rate):
        rospy.init_node('move_robot', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_pose)
        self.pose_odom = rospy.Subscriber('/odom', Odometry, self.current_angle)
        self.goal_rotate_tolerance = 0.25 #rad
        self.goal_rotate_tolerance_check = 0.1
        self.goal_tolerance_linear = 0.35
        self.position_tolerance_linear = 0.35
        self.twist_msg = Twist()
        self.pose = Odometry()
        self.rate = rospy.Rate(rate)  # Set the rate based on the argument
        self.velocity = velocity
        self.ang_velocity = 0.2
        self.yaw = 0
        self.next_straight = False

    def current_angle(self,data):
        odom_angle=data.pose.pose.orientation
        _, _, self.odom_angle = euler_from_quaternion([odom_angle.x, odom_angle.y, odom_angle.z, odom_angle.w])
        
    def next_point(self, goal_pose, next_point):
        x_goal = goal_pose.pose.position.x
        y_goal = goal_pose.pose.position.y

        x_act = self.pose.pose.pose.position.x
        y_act = self.pose.pose.pose.position.y
        print("Actual position",x_act,y_act)
        print("Goal",x_goal,y_goal)
        
        if next_point:
            next_x_goal = next_point.pose.position.x
            next_y_goal = next_point.pose.position.y

            print("Next goal",next_x_goal,next_y_goal)
            angle_to_next_goal = math.atan2(y_goal - y_act, x_goal - x_act)
            
            x_diff = x_goal - next_x_goal
            y_diff = y_goal - next_y_goal
            print("Differences",x_diff,y_diff,abs(angle_to_next_goal - self.yaw))
                
            if(((x_diff == 0) or (y_diff == 0)) and (abs(angle_to_next_goal - self.yaw) < self.goal_rotate_tolerance_check)):
                self.next_straight = True
            else:
                self.next_straight = False

        print("········",self.next_straight)

    def calculate_angle(self, goal_pose):
        try:
            if(not self.next_straight):
                rospy.loginfo(f"Rotating to correct angle...")

                x_act = self.pose.pose.pose.position.x
                y_act = self.pose.pose.pose.position.y
                z_act = self.pose.pose.pose.orientation

                current_angle=self.odom_angle
                    
                _, _, self.yaw = euler_from_quaternion([z_act.x, z_act.y, z_act.z, z_act.w])

                x = goal_pose.pose.position.x
                y = goal_pose.pose.position.y
            
                angle_to_next_goal = math.atan2(y - y_act, x - x_act)
                rospy.loginfo(angle_to_next_goal)

                print("Angles in",angle_to_next_goal,self.yaw,angle_to_next_goal - self.yaw)
                while (abs(angle_to_next_goal - current_angle) > self.goal_rotate_tolerance) and (not rospy.is_shutdown()):
                    
                    z_act = self.pose.pose.pose.orientation
                    _, _, self.yaw = euler_from_quaternion([z_act.x, z_act.y, z_act.z, z_act.w])
                    
                    rospy.loginfo(abs(angle_to_next_goal - self.yaw))
                    # Calculate angular velocity to rotate towards the next goal point
                    angular_velocity = (angle_to_next_goal - self.yaw) * self.ang_velocity
                    if angular_velocity > self.ang_velocity:
                        angular_velocity = self.ang_velocity

                    # Create Twist message to rotate the robot
                    self.twist_msg.angular.z = angular_velocity
                    self.velocity_publisher.publish(self.twist_msg)

                    current_angle = self.odom_angle
                    # rospy.loginfo(self.odom_angle)
                    # while((abs(current_angle-self.odom_angle)<(angle_to_next_goal - self.yaw)-.05) and
                    #       (not rospy.is_shutdown() )):
                    #     rospy.loginfo(f"current_angle diff: {abs(current_angle-self.odom_angle)}")
                    #     rospy.sleep(0.03)
                    # self.stop_robot()
                    # break
                    
                    # z_act = self.pose.pose.pose.orientation
                    # _, _, self.yaw = euler_from_quaternion([z_act.x, z_act.y, z_act.z, z_act.w])
                    # while(self.yaw==old_angle and (not rospy.is_shutdown())):
                    #     z_act = self.pose.pose.pose.orientation
                    #     _, _, self.yaw = euler_from_quaternion([z_act.x, z_act.y, z_act.z, z_act.w])
                    #     rospy.loginfo("waiting new amcl")
                    #     rospy.sleep(0.03)

                
                self.twist_msg.angular.z = 0
                self.velocity_publisher.publish(self.twist_msg)  # Stop the robot
                print("Angles out",angle_to_next_goal,self.yaw,angle_to_next_goal - self.yaw)
                rospy.loginfo(f"Angle riched!!!")
            else:
                rospy.loginfo(f"Going straigth")
       
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def linear_movement(self, goal_pose, index):
        try:
            x_goal = goal_pose.pose.position.x
            y_goal = goal_pose.pose.position.y

            x_act = self.pose.pose.pose.position.x
            y_act = self.pose.pose.pose.position.y

            x_diff = x_act - x_goal
            y_diff = y_act - y_goal

            distance_to_goal = math.sqrt((x_diff)**2 + (y_diff)**2)

            rospy.loginfo("Driving to next spot...")
            while True and (not rospy.is_shutdown()):
                x_act = self.pose.pose.pose.position.x
                y_act = self.pose.pose.pose.position.y

                x_diff = x_act - x_goal
                y_diff = y_act - y_goal

                distance_to_goal = math.sqrt((x_diff)**2 + (y_diff)**2)

                # Calculate linear velocity
                if not self.next_straight:
                    vel = self.velocity * distance_to_goal
                    if vel > self.velocity:
                        vel = self.velocity
                else:
                    vel = self.velocity

                # Calculate angular velocity for orientation correction
                angle_to_goal = math.atan2(y_goal - y_act, x_goal - x_act)
                angular_velocity = (angle_to_goal - self.yaw) * self.velocity

                # Publish twist message with linear and angular velocity
                self.twist_msg.linear.x = vel
                self.twist_msg.linear.y = vel
                self.twist_msg.linear.z = 0
                #if (abs(angle_to_goal - self.yaw) > self.goal_rotate_tolerance):
                #    self.twist_msg.angular.z = angular_velocity
                #else:
                    #self.twist_msg.angular.z = 0
                self.velocity_publisher.publish(self.twist_msg)

                # Check if the robot is close to the goal position
                if abs(distance_to_goal) < self.goal_tolerance_linear:
                    # Check if the robot's actual position is similar to the goal position
                    if (abs(x_diff) < self.position_tolerance_linear and
                        abs(y_diff) < self.position_tolerance_linear):
                        if not self.next_straight:
                            self.twist_msg.linear.x = 0
                            self.twist_msg.linear.y = 0
                            self.twist_msg.angular.z = 0
                            self.velocity_publisher.publish(self.twist_msg)
                            rospy.loginfo("Stopped")
                        else:
                            rospy.loginfo("Going straight")
                        break

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    
    def update_pose(self, data):
        self.pose = data

    def stop_robot(self):
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = 0
        self.twist_msg.angular.z = 0
        self.twist_msg.angular.z = 0
        self.velocity_publisher.publish(self.twist_msg)
        
    def get_current_position(self):
        return self.pose.pose.pose.position.x, self.pose.pose.pose.position.y, self.yaw
    
    def navigate_to_next_goal(self, point, next_goal_point=None, index=0):
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = point[0]
        goal_pose.pose.position.y = point[1]
        print(index)
        self.next_point(goal_pose, next_goal_point)
        self.calculate_angle(goal_pose)
        self.linear_movement(goal_pose, index)

    def update_pose_callback(data, move_robot_instance):
        move_robot_instance.update_pose(data)
        global pose_gl 
        pose_gl = data

    def make_service_request(self,start_point, end_point):
        rospy.wait_for_service('path_planner')  # Wait for the service to become available
        try:
            service_client = rospy.ServiceProxy('path_planner', path)
            request = pathRequest(start_point=start_point, end_point=end_point)
            response = service_client(request)
            return response.path
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return None
    
    def move_robot(self, goal):

        start_x,start_y,yaw = self.get_current_position()
        
        # Initialize previous goal pose as None
        next_goal_point = None
        start_point = [start_x,start_y]

        path = self.make_service_request(start_point, goal)
        j = 0

        while True:   
            for i in range(len(path)):
                #If ros is shutdown exit function
                if rospy.is_shutdown():
                    return

                point = [path[i].location[0], path[i].location[1]]
                next_point = path[i + 1] if i + 1 < len(path) else None
                
                if(next_point):
                # Update the previous goal pose
                    next_goal_point = PoseStamped()
                    next_goal_point.pose.position.x = next_point.location[0]
                    next_goal_point.pose.position.y = next_point.location[1]
                else:
                    next_goal_point = None
                # Navigate to the next goal, passing the previous goal pose
                move_robot.navigate_to_next_goal(point, next_goal_point,i) 
                print("##################################")
                if(i == 10):
                    break
            
            self.stop_robot()
            curr_x, curr_y,_ = self.get_current_position()
            start_point = [curr_x,curr_x]
            if((abs(goal[0]-curr_x) < self.goal_tolerance_linear) and 
               (abs(goal[1]-curr_y) < self.goal_tolerance_linear)):
                break
            path = self.make_service_request(start_point, goal)
            


if __name__ == '__main__':
    end_point = [4, 3]
    velocity = 0.5
    rate = 10
    move_robot = MoveRobot(velocity, rate)  # Pass the rate to the constructor
    move_robot.move_robot(end_point)
