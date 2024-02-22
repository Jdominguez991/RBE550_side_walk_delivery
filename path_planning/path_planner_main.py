from algorithms import *
from algorithm_publisher_subscriber import *
from gazebo_msgs.msg import ModelStates
import rospy


if __name__ == "__main__":
    # example move command to the robot
    rospy.init_node('robot_position_listener', anonymous=True)
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
    rospy.spin()
    start = [robot_position.x, robot_position.y, robot_position.z]
    goal = [2, 0]  # must change to whatever the server is asking for based on next item in the list
    map = request_map()
    time.sleep(1)
    grid = map_array(map)
    robot_planner = Algorithms(start, goal, obstacles, grid, rand_area, expand_dis=0.5, goal_sample_rate=20, max_iter=2000)


