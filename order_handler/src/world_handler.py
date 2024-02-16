#!/usr/bin/env python
import subprocess
import os
import yaml
import gazebo_msgs
import geometry_msgs
from gazebo_msgs.srv import SpawnModelRequest, SpawnModel
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Point, Pose, Quaternion
import roslaunch
import rospy
from yaml.loader import SafeLoader
from tf.transformations import quaternion_from_euler
import argparse
from pathlib import Path

def spawn_item(name:str,location,sdf_file:Path,trial_cnts=0):
    """Will spawn the given item into the world

    Args:
        name (String): The name the model will be called
        location ([x,y,rotation]): The location and rotation in the z axis the model will have when spawned
        sdf_file (Path): A path to the sdf file with the model
        trial_cnts (int): The number of times a product has tried to be spawned. Defaults to 0.
    """
    global vars_dict
    # Create the spawn service
    spawn_service = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    #Make the spawn location of type Pose will spawn everything at 1 meter high
    dispatch_location=Pose()
    dispatch_location.position.x=location[0]
    dispatch_location.position.y=location[1]
    dispatch_location.position.z=1
    quat=quaternion_from_euler(0,0,location[2])
    dispatch_location.orientation=Quaternion(quat[0],quat[1],quat[2],quat[3])

    #Grab the sdf information from the file
    if not sdf_file.exists():
        rospy.logerr(f"Invalid sdf file given {sdf_file}")
        return
    f= open(str(sdf_file),'r')
    sdf = f.read()
    f.close()

    #create the msg
    msg=SpawnModelRequest()
    msg.model_name=name
    msg.model_xml=sdf   
    msg.initial_pose=dispatch_location
    msg.reference_frame="world"
    rospy.logdebug(f"name: {msg.model_name} sdf_file: {sdf_file} location: {location}")

    #Wait until the the service is avail
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    try:
        #Try to spawn the model if not able to try to do it again assuming it never hit the exception
        resp=spawn_service(msg)
        if(not resp.success):
            rospy.logwarn(f"dispatch {name} not spawned waiting 5 seconds and trying again")
            rospy.sleep(5)
            #If trial 
            current_attempts=trial_cnts+1
            if(current_attempts>=vars_dict['max_spawn_trials']):
                rospy.logerr(f"tried to spawn model {name} {vars_dict['max_spawn_trials']} times and all trials failed, giving up")
                return
            else:
                spawn_item(name=name,location=location,sdf_file=sdf_file)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def make_pickup_loc(name:str,location,color:str):
    """A function to select the correct sdf model based on the color

    Args:
        name (String): The name the model will be called
        location ([x,y,rotation]): The location and rotation in the z axis the model will have when spawned
        color (String): A single char 
    """
    if color.upper()=="B" or color.upper()=="R" or color.upper()=="G":
        model=f"Storage{color.upper()}"
        sdf_file=(Path(__file__).resolve().parents[0]).joinpath(f"../model/warehouseObjects/{model}.sdf")
        if not sdf_file.exists():
            rospy.logerr(f"The color given does not have a file for it path to file {sdf_file}")
        else:
            spawn_item(name=name,location=location,sdf_file=sdf_file)
    else:
        rospy.logerr(f"invalid color given for product pick up color given: {color.upper()}")


parser = argparse.ArgumentParser(description ='Process some integers.')
parser.add_argument('--spawn_world', help ='Should the file launch the world',type=bool,default=True)
parser.add_argument('--world', help ='Which world launch file do you want to use', default="world.launch")
parser.add_argument('--spawn_drop_off', help ='Should the drop off locations be spawned', default=True)
parser.add_argument('--spawn_pick_up', help ='Should the pickup locations be spawned', default=True)
parser.add_argument('--max_spawn_trials', help ='The number of times the sim will try to spawn the item', default=5)
 
args, unknown = parser.parse_known_args()
vars_dict=vars(args)

rospy.init_node('world_spawner', log_level=rospy.INFO)

#Spawn the world based on the world launch file 
if vars_dict['spawn_world']:
    rospy.logdebug("World being launched")
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [str((Path(__file__).resolve().parents[0]).joinpath(f"../launch/{vars_dict['world']}"))])
    launch.start()
    rospy.logdebug("world launched")
    rospy.sleep(10)


node_launch = roslaunch.scriptapi.ROSLaunch()
# Open the cfg file and load the file
with open(f'{os.path.dirname(os.path.abspath(__file__))}/conf.yaml','r') as f:
    data = yaml.load(f, Loader=SafeLoader)
    #Spawn in the drop off stations
    if vars_dict['spawn_drop_off']:
        for depo in data["drop_off_loc"]:
            spawn_item(name=depo["name"],location=depo["spawn_location"],sdf_file=(Path(__file__).resolve().parents[0]).joinpath("../model/warehouseObjects/Dispatch.sdf"))
    #spawn in the pick up stations
    if vars_dict['spawn_pick_up']:
        for pickup in data["pick_up_loc"]:
            make_pickup_loc(name=pickup["name"],location=pickup["unit_location"],color=pickup["color"])
            node=roslaunch.core.Node("order_handler", "product_spawner.py",args=f"-x_pos={pickup['product_spawn_location'][0]} -y_pos={pickup['product_spawn_location'][1]} -color={pickup['color']}")
            node_launch.start()
            node_launch.launch(node)

rospy.spin()
# 3 seconds later
# launch.shutdown()