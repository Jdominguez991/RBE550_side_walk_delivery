#!/usr/bin/env python
import rospy
import sys
import argparse
from gazebo_msgs.srv import GetModelStateRequest, GetModelState,GetModelPropertiesResponse
from std_srvs.srv import SetBool,SetBoolResponse,SetBoolRequest, Trigger, TriggerResponse
from gazebo_msgs.srv import SpawnModelRequest, SpawnModel
from geometry_msgs.msg import Point, Pose, Quaternion
from tf.transformations import quaternion_from_euler
import os
from pathlib import Path
from robot_operation.srv import send_order, send_orderResponse
import math
import numpy as np
product_cnt=0


def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

def spawn_product(trial_cnts=0):
    """Spawn a product in the world

    Args:
        trial_cnts (int): The number of times a product has tried to be spawned. Defaults to 0.

    Returns:
        String: Will return the ID of the product or return nothing indicating spawn failed
    """
    global vars_dict,product_cnt
    #Create the spawn service
    spawn_service = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    # Make the given location a Pose type
    dispatch_location=Pose()
    dispatch_location.position.x=vars_dict['x_pos']
    dispatch_location.position.y=vars_dict['y_pos']
    dispatch_location.position.z=vars_dict['z_pos']
    quat=get_quaternion_from_euler(0,0,0)
    dispatch_location.orientation=Quaternion(quat[0],quat[1],quat[2],quat[3])

    #Grab the sdf information from the file
    sdf_file=(Path(__file__).resolve().parents[0]).joinpath(f"../model/warehouseObjects/Product{vars_dict['color'].upper()}.sdf")
    if not sdf_file.exists():
        rospy.logerr(f"Invalid sdf file given {sdf_file}")
        return "Invalid file"
    f= open(str(sdf_file),'r')
    sdf = f.read()
    f.close()

    #Create the msg
    msg=SpawnModelRequest()
    msg.model_name=f"{vars_dict['color']}_{product_cnt}"
    msg.model_xml=sdf   
    msg.initial_pose=dispatch_location
    msg.reference_frame="world"
    rospy.logdebug(f"name: {msg.model_name} sdf_file: {sdf_file} location: {msg.pose.position}")

    #Wait until service is avail
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    try:
        resp=spawn_service(msg)
        #try to respawn the model if not successful try at most 5 times
        if(not resp.success):
            rospy.logwarn(f"dispatch {msg.model_name} not spawned waiting 5 seconds and trying again")
            rospy.sleep(5)
            current_attempts=trial_cnts+1
            if(current_attempts>=vars_dict['max_spawn_trials']):
                rospy.logerr(f"tried to spawn model {msg.model_name} {vars_dict['max_spawn_trials']} times and all trials failed, giving up")
                return ""
            else:
                spawn_product(current_attempts)
        else:
            return msg.model_name
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def grab_curr_robot_pose():
    """Call the get_model_state of the delivery robot and grab its current x and y location

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
    
def check_spawn(data):
    """This gets called when ever the the service is requested to spawn a product. It will only try spawning a product is the robot is close enough to the spawn location

    Args:
        data : The data required for ros to use this function as a service this service type does not have any input data

    Returns:
        TriggerResponse: A msg with the if spawn was successful and the if of the spawned item
    """
    global vars_dict
    return_msg=TriggerResponse()
    robot_location=grab_curr_robot_pose()
    if (robot_location[0] in range(vars_dict['x_pos']-vars_dict['rbt_tol'],vars_dict['x_pos']-vars_dict['rbt_tol'])) and (robot_location[0] in range(vars_dict['y_pos']-vars_dict['rbt_tol'],vars_dict['y_pos']-vars_dict['rbt_tol'])):
        id=spawn_product()
        return_msg.success=True
        return_msg.message=id
    else:
        rospy.logwarn(f"robot not in position current robot pos for product spawn color: {vars_dict['color']} current_loc: {robot_location}")
        rospy.logwarn(f"x needs to be between {vars_dict['x_pos']-vars_dict['rbt_tol']} and  {vars_dict['x_pos']+vars_dict['rbt_tol']} y needs to be between {vars_dict['y_pos']-vars_dict['rbt_tol']} and  {vars_dict['y_pos']+vars_dict['rbt_tol']}")
        return_msg.success=False
        return_msg.message="Not in Position"
    return return_msg


parser = argparse.ArgumentParser(description ='Process some integers.')
parser.add_argument('-x_pos', help ='x position the object will be spawned')
parser.add_argument('-y_pos', help ='y position the object will be spawned')
parser.add_argument('-color', help ='The color the product should be')
parser.add_argument('--rbt_tol', type=float,help ='The color the product should be',default=0.25)
parser.add_argument('--z_pos', help ='z position the object will be spawned', default=.5)
parser.add_argument('--max_spawn_trials', help ='The number of times the sim will try to spawn the item', default=5)
 
args, unknown = parser.parse_known_args()
vars_dict=vars(args)
rospy.init_node(f'product_spawner_color', log_level=rospy.INFO)
s = rospy.Service(f'product_spawner_{vars_dict["color"]}', Trigger, check_spawn)
rospy.logdebug("product spawner ready",logger_name="my_logger_name")
rospy.spin()

