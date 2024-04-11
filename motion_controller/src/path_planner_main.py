#!/usr/bin/env python

import algo_functions.algorithms as algorithms
import algo_functions.algorithm_publisher_subscriber as algo_functions
import rospy
import math
import tf
import numpy
from scipy.spatial.transform import Rotation
from gazebo_msgs.srv import GetModelStateRequest, GetModelState,GetModelPropertiesResponse
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import GridCells, Odometry
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
from scipy import ndimage

start_location={"x":0,"y":0,"angle":0}
end_location={"x":0,"y":0,"angle":0}
given_coor=[0,0]

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

        self.calculate_angle(goal_pose)
        self.linear_movement(goal_pose)
        self.rate.sleep()

def grab_curr_robot_pose():
    """Call the get_model_state of the delivery robot and grab its current x and y start_location

    Returns:
        [x,y]: The x and y position of the robot
    """
    #Create the service
    model_info = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    #Create the msg
    msg=GetModelStateRequest()
    msg.model_name="delvery_robot"

    #Wait until service is avail if trying to be called early
    rospy.wait_for_service("/gazebo/get_model_state")
    try:
        #Break the msg down and grab the robots current x and y pose
        info=model_info(msg)
        return [info.pose.position.x,info.pose.position.y]
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def starting_pnt(data):
    global start_location, given_coor
    start_location["x"]=int(data.pose.pose.position.x/.05)
    start_location["y"]=int(data.pose.pose.position.y/.05)
    quaternion_angle=[data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
    
    rot = Rotation.from_quat(quaternion_angle)
    start_location["angle"]= rot.as_euler('xyz', degrees=True)
    # print(start_location)
    rospy.loginfo(f"start coordinates:{start_location}")
    given_coor[0]=1
def end_pnt(data):
    global start_location, given_coor
    end_location["x"]=int(data.pose.position.x/.05)
    end_location["y"]=int(data.pose.position.y/.05)
    quaternion_angle=[data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w]
    
    rot = Rotation.from_quat(quaternion_angle)
    end_location["angle"]= rot.as_euler('xyz', degrees=True)
    rospy.loginfo(f"end coordinates:{end_location}")
    given_coor[1]=1
    
if __name__ == "__main__":
    
    rospy.init_node('path_planner',anonymous = True, log_level=rospy.DEBUG)
    rospy.Subscriber('initialpose', PoseWithCovarianceStamped, starting_pnt)
    rospy.Subscriber('move_base_simple/goal', PoseStamped, end_pnt)
    cSpacePub = rospy.Publisher('/resulting_path', GridCells, queue_size=10)
    start_end_pntPub = rospy.Publisher('/start_end_pnt', GridCells, queue_size=10)

    print("getting map")
    grid= algo_functions.request_map()

    # transform 1D map data to 2D array
    print(len(grid[0]))
    count=0
    occu_map_array=[]
    for index in range(0,grid[1]):
        temp_array=[]
        for h_index in range(0,grid[2]):
            temp_array.append(grid[0][count])
            count+=1
        occu_map_array.append(temp_array)
    print(f'Dimension of the occupancy grid array: {len(occu_map_array)}')

    arr = numpy.array(occu_map_array)
    rotated_array=numpy.rot90(arr)
    rotated_array=numpy.flip(rotated_array,0)
    
    convert_values=rotated_array
    convert_values[convert_values<0] = 0

    # Taking a matrix of size 5 as the kernel 
    kernel = numpy.ones((5, 5), numpy.uint8)
    struct1 = ndimage.generate_binary_structure(2, 1)
    convert_values=ndimage.binary_dilation(convert_values, structure=struct1,iterations=2).astype(convert_values.dtype)
    # rotated_array = cv2.dilate(rotated_array, kernel, iterations=1)  

    copy_original_map=rotated_array
    for iy, ix in numpy.ndindex(convert_values.shape):
        if convert_values[iy,ix]==1 and not copy_original_map==1:
            rotated_array[iy,ix]=.7


    rate = rospy.Rate(15)


    # print(arr)
    # path_array=[]
    # for num_row,value in enumerate(rotated_array):
    #     for num_col,col_value in enumerate(value):
    #         if not col_value == -1 and not col_value==0:
    #             #Display points that it has visited
    #             print(num_col, num_row, col_value)
    #             highlight=Point()
    #             highlight.x=(num_row-1000)*.05+.025         # translate from grid space coordinate to world space coordinate
    #             highlight.y=(num_col-1000)*.05+.025
    #             highlight.z=0
    #             path_array.append(highlight)

    #             grid_cells_msg = GridCells()
    #             grid_cells_msg.cell_width = grid[3]
    #             grid_cells_msg.cell_height = grid[3]
    #             grid_cells_msg.cells = path_array
    #             grid_cells_msg.header.frame_id = "map"
    #             cSpacePub.publish(grid_cells_msg)
    
    # for w in range(1,5):
    #     cSpacePub.publish(grid_cells_msg)
    #     rate.sleep()

    # sys.exit()

    rate = rospy.Rate(15)
    while(given_coor[0]==0 or given_coor[1]==0):
        rate.sleep()

    start = [start_location["x"],start_location["y"]]                
    goal = [end_location["x"],end_location["y"]]                                   # currently arbitrary, ust change to whatever the server is asking for based on next item in the list
    
    # start=[0,0]
    # goal=[50,50]

    

    start_end_pnt=[]
    highlight=Point()
    highlight.x=(start[0])*.05+.025         # translate from grid space coordinate to world space coordinate
    highlight.y=(start[1])*.05+.025
    highlight.z=0
    start_end_pnt.append(highlight)

    highlight=Point()
    highlight.x=(goal[0])*.05+.025         # translate from grid space coordinate to world space coordinate
    highlight.y=(goal[1])*.05+.025
    highlight.z=0
    start_end_pnt.append(highlight)

    grid_cells_msg = GridCells()
    grid_cells_msg.cell_width = grid[3]
    grid_cells_msg.cell_height = grid[3]
    grid_cells_msg.cells = start_end_pnt
    grid_cells_msg.header.frame_id = "map"
    start_end_pntPub.publish(grid_cells_msg)

    # initialize path planning object
    dimension = len(arr)                                                 # occupancy grid is square
    rand_area = [1,2]                                                              # for RRT later
    robot_planner = algorithms.Algorithms(start,goal,dimension,dimension,list(rotated_array), rand_area)    # create path planning object
    
    # checkpoint testing
    check_order = robot_planner.checkpoint_order()
    print(f'These are the checkpoints: {check_order}')               # print the checkpoint list
    print(f'This is the checkpoint order')

    robot_planner.a_star()                                                       # call a_star method, no expected return
    # print(f"This is the A_star path: {robot_planner.path['A_star']}")                           # access A_star key to display path
    

    move_robot = MoveRobot()
    velocity = 0.5

    path_array=[]
    print('Sending A* to rviz')
    for value in robot_planner.path['A_star']:
        print(value)                        # print path
        #Display points that it has visited
        highlight=Point()
        highlight.x=(value[0]-1000)*.05+.025         # translate from grid space coordinate to world space coordinate
        highlight.y=(value[1]-1000)*.05+.025
        highlight.z=0
        path_array.append(highlight)

        grid_cells_msg = GridCells()
        grid_cells_msg.cell_width = grid[3]
        grid_cells_msg.cell_height = grid[3]
        grid_cells_msg.cells = path_array
        grid_cells_msg.header.frame_id = "map"
        cSpacePub.publish(grid_cells_msg)
    
    for points in path_array:
        print(points)
    # Define goal pose
    #goal_pose = PoseStamped()
    #goal_pose.pose.position.x = x_goal
    #goal_pose.pose.position.y = y_goal

    #move_robot.move2goal(goal_pose, velocity)

    for w in range(1,5):
        cSpacePub.publish(grid_cells_msg)
        rate.sleep()

    rospy.spin()

