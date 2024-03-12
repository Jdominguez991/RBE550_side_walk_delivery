import algo_functions.algorithms as algorithms
import algo_functions.algorithm_publisher_subscriber as algo_functions
from gazebo_msgs.msg import ModelStates
import rospy
from scipy.spatial.transform import Rotation
from gazebo_msgs.srv import GetModelStateRequest, GetModelState,GetModelPropertiesResponse
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import GridCells
import numpy 

start_location={"x":0,"y":0,"angle":0}
end_location={"x":0,"y":0,"angle":0}
given_coor=[0,0]
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
    robot_planner.a_star()                                                       # call a_star method, no expected return
    print(f"This is the A_star path: {robot_planner.path['A_star']}")                                          # access A_star key to display path
    
    path_array=[]
    for value in robot_planner.path['A_star']:
        print(value)
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
    
    for w in range(1,5):
        cSpacePub.publish(grid_cells_msg)
        rate.sleep()

    rospy.spin()

