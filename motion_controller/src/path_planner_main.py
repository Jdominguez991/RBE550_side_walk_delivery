from algorithms import *
from algorithm_publisher_subscriber import *
from gazebo_msgs.msg import ModelStates
import rospy
from scipy.spatial.transform import Rotation
from gazebo_msgs.srv import GetModelStateRequest, GetModelState,GetModelPropertiesResponse
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion

start_location={"x":0,"y":0,"angle":0}
end_location={"x":0,"y":0,"angle":0}
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
    global start_location
    start_location["x"]=data.pose.position.x
    start_location["y"]=data.pose.position.y
    quaternion_angle=[data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w]
    
    rot = Rotation.from_quat(quaternion_angle)
    start_location["angle"]= rot.as_euler('xyz', degrees=True)
    print(start_location)
def end_pnt(data):
    global start_location
    end_location["x"]=data.pose.position.x
    end_location["y"]=data.pose.position.y
    quaternion_angle=[data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w]
    
    rot = Rotation.from_quat(quaternion_angle)
    end_location["angle"]= rot.as_euler('xyz', degrees=True)
    print(end_location)
    
if __name__ == "__main__":
    
    rospy.init_node('path_planner',anonymous = True)
    rospy.Subscriber('initialpose', PoseStamped, starting_pnt)
    rospy.Subscriber('move_base_simple/goal', PoseStamped, end_pnt)
    
    
    # start = [start_location["x"],start_location["y"],start_location["angle"]]                
    # goal = [2, 0]                                   # currently arbitrary, ust change to whatever the server is asking for based on next item in the list
    # map = [request_map()]
    # time.sleep(1)
    # grid = map_array(map)
    # rand_area = 0

    # filename = 'map1.pgm'
    # robot_planner = Algorithms(start, goal,rand_area,filename, expand_dis=0.5, goal_sample_rate=20, max_iter=2000)
    # a_star_path = robot_planner.a_star()

    rospy.spin()

