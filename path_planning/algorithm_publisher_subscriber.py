import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path

             # import everything from algorithms
def model_states_callback(msg):
    # Find the index of your robot in the list of model names
    try:
        robot_index = msg.name.index("your_robot_name")             # MUST CHANGE TO ROBOT NAME
    except ValueError:
        rospy.logwarn("Robot not found in model states")
        return

    # Extract position and orientation of the robot
    robot_pose = msg.pose[robot_index]
    robot_position = robot_pose.position
    robot_orientation = robot_pose.orientation

    # Process the position and orientation as needed
    rospy.loginfo("Robot position: x={}, y={}, z={}".format(robot_position.x, robot_position.y, robot_position.z))
    rospy.loginfo("Robot orientation: x={}, y={}, z={}, w={}".format(robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w))

    return robot_pose,robot_position, robot_orientation


# do we need a move command with path planner?
def move(position):
    rospy.init_node('robot_mover', anonymous=True)

    # Create publisher
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

    # Create message
    state_msg = ModelState()
    state_msg.model_name = "your_robot_name"
    state_msg.pose.position = position
    state_msg.pose.orientation.x = 0.0
    state_msg.pose.orientation.y = 0.0
    state_msg.pose.orientation.z = 0.0
    state_msg.pose.orientation.w = 1.0
    state_msg.twist.linear = Twist()
    state_msg.twist.angular = Twist()

    # Publish message
    pub.publish(state_msg)
# rospy.init_node('robot_position_listener', anonymous=True)
# rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
# rospy.spin()


# create a function for getting a occupancy map
def request_map():
    global resolution_cell_per_meter
    """
    Requests the map from the map server.
    :return [OccupancyGrid] The grid if the service call was successful,
                                                                                                    None in case of error.
    """
    # REQUIRED CREDIT
    rospy.loginfo("Requesting the map")
    # grid = rospy.ServiceProxy('nav_msgs/GetMap', GetMap)
    rospy.wait_for_service('/static_map')
    try:
        grid = rospy.ServiceProxy('/static_map', GetMap)
        occuGrid = grid()
        return(occuGrid.map)
    except:
        rospy.loginfo("Failed")
def map_array(map):
    data=map.data
    item_num=0
    final_array=[]
    for x in range(0,128):
        new_y=[]
        for  item in range(0,128):
            new_y.append(data[item_num])
            item_num+=1
        final_array.append(new_y)
    return final_array