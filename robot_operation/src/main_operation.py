#!/usr/bin/env python

from statemachine import StateMachine, State
import rospy
from robot_operation.srv import send_order, send_orderResponse
from std_msgs.msg import String
from pathlib import Path
from yaml.loader import SafeLoader
import yaml
from threading import Thread
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_srvs.srv import SetBool,SetBoolResponse,SetBoolRequest, Trigger, TriggerResponse
import time
import move_base.move_base as move_base
import re
import sys

class SingletonClass(object):
    """A class to define a class to be singleton

    Args:
        object (_type_): to say the class should be treated as an object
    """
    def __new__(cls):
        if not hasattr(cls, 'instance'):
            cls.instance = super(SingletonClass, cls).__new__(cls)
        return cls.instance
class acml(SingletonClass):
    """The class for amcl to determine if the robot is localized used to say when the robot can accept orders

    Args:
        SingletonClass (_type_):Extend from singlton class as there should only be one amcl class 
    """
    def is_localized():
        """Function to determine to return if the robot is localized

        Returns:
            bool: For if the robot is localized 
        """
        rospy.logdebug("default robot always localized need to change when navigation code is made")
        return True
class main_states(StateMachine):
    """The state machine for what state the robot is currently in

    Args:
        StateMachine (_type_): The class that it should be extend from
    """
    waiting_order = State(initial=True)
    picking_up_items = State()
    moving_to_drop_off = State()
    moving_back_home = State()

    step = (
        waiting_order.to(picking_up_items)
        | picking_up_items.to(moving_to_drop_off)
        | moving_to_drop_off.to(moving_back_home)
        | moving_back_home.to(waiting_order)
    )

    def before_step(self, event: str, source: State, target: State):
        """Used to change the state it is currently on

        Args:
            event (str): The event that should happen
            source (State): The original state before transition
            target (State): The new state to be used

        Returns:
            _type_: A string stating from start id and end id
        """

        # message = ". " + message if message else ""
        rospy.loginfo(f"Running {event} from {source.id} to {target.id}")


class current_order_status(SingletonClass):
    """Class to handle how the ordering should be handled 
    like the location of the depositories, as well holding 
    information related to the order it is working on.

    Args:
        SingletonClass (_type_): Define this as a singleton class
    """
    #[quantity, is_picked_up]
    object_info={"red":[0,True,[0,0]],"green":[0,True,[0,0]],"blue":[0,True,[0,0]]} #If set to true means no items from that color need to be picked up
    end_location=[0,0] #x,y
    desired_points=[]
    home_position=[0,0]
    current_robot_state=main_states()

    def reset_states(self):
        """Reset the quantity of a all the colors
        """
        copy_info= self.object_info
        for color in copy_info.keys():
            self.object_info[color][0]=0
            self.object_info[color][1]=True

    def __init__(self):
        config_yaml=(Path(__file__).resolve().parents[2]).joinpath('order_handler/src/conf.yaml')     
        # Read the config yaml to grab the location the item will be spawned in
        with open(config_yaml,'r') as f:
            data = yaml.load(f, Loader=SafeLoader)
            # iterate through each pick up location item to update where the pick of all the colors is
            for pickup in data["pick_up_loc"]:
                if pickup["color"].upper() == "R":
                    self.object_info["red"][2]=pickup["unit_location"][:2]
                elif pickup["color"].upper() == "G":
                    self.object_info["green"][2]=pickup["unit_location"][:2]
                elif pickup["color"].upper() == "B":
                    self.object_info["blue"][2]=pickup["unit_location"][:2]
                else:
                    rospy.logerr("unknown color, please fix if statement for new color")
            
            # Grab the location the robot would spawn in
            self.home_position=data["robot_info"]["location"]

    def incoming_order(self,r: str,g: int,b: int,location: str):
        """Assign The incoming order and update the quantity 

        Args:
            r (str): number of red items
            g (int): number of green items
            b (int): number of blue items
            location (str): The end location of the item should be delivered to

        Returns:
            bool : Always return true so that node can service can return order received
        """
        # Set all value back to zero
        self.reset_states()

        # put the quantity of each item into dictionary and say now of the items are picked
        if(r>0):
            self.object_info["red"][0]=[r,False]
        if(g>0):
            self.object_info["green"][0]=[g,False]
        if(b>0):
            self.object_info["blue"][0]=[b,False]

        # Split string of input to x and y pos and put into end position
        split_location=re.findall(r"\d+", location)
        print(split_location)
        self.end_location=[int(split_location[0]),int(split_location[1])]

        # make array of points to visited
        self.__process_points()

        #Move the state machine to the next state
        self.current_robot_state.step()
        return True

    def __process_points(self):
        """Make array of points the robot needs to visit input of each item if [[x,y],<go next step>]
        """

        # go through each color to see if robot needs to pick up item
        for item_num,info in enumerate(self.object_info):
            rospy.loginfo(self.object_info[info][0][0])
            if self.object_info[info][0][0] > 0:
                if item_num==0:
                    self.desired_points.append([self.object_info[info][2],1])
                else:
                    self.desired_points.append([self.object_info[info][2],0])

        # Add end location as well as add the home position
        self.desired_points.append([self.end_location,1])
        self.desired_points.append([self.home_position,1])

    def new_point(self):
        """Grab a new point

        Returns:
            2x1 int array: return 2x1 array of x an y position of next point to be reached
        """
        # Pop the front item and grab the next front item 
        self.desired_points.pop(0)

        #   Check if there if the value outside of coordinate says go to next state
        try:
            if self.desired_points[0][1]:
                self.current_robot_state.step()
            return self.desired_points[0][0]
        except:
            self.current_robot_state.step()
            return None
    
    def current_goal(self):
        """The current goal the robot should drive to

        Returns:
            2x1 int array[x,y]: Current goal to drive to
        """
        return self.desired_points[0]
    
    def desired_points_list(self):
        """entire list of all the point robot will try to drive to

        Returns:
            list of points: All the points the robot will try to go to 
        """
        return self.desired_points

    def current_state(self):
        """return the current state the robot is in

        Returns:
            string: The current state the robot is in
        """
        return self.current_robot_state.current_state.id

class ros_class:
    order_info=current_order_status()

    def received_order(self,data):
        """function for when the order service gets called, 
        This is used to grab the msg data and decode it into the order status class

        Args:
            data (send_order.srv): Data being sent over the service

        Returns:
            send_orderResponse: Say msg was received and acknowledge
        """
        self.order_info.incoming_order(data.r,data.g,data.b,data.location)
        return send_orderResponse(True)
    
    #Keep sending robot status for any one that wants it
    def keep_sending_status(self,event=None):
        """Publish the current robot status over the publisher current status for as long as the sim is running
        """

        rospy.logdebug(f"publishing state: {self.order_info.current_state().upper()}")
        self.status.publish(self.order_info.current_state().upper())
    def __init__(self):
        """What to do when starting this class
        """
        
        # Define service for getting new information for order
        self.s = rospy.Service('robot1/receive_order', send_order, self.received_order)
        # Publisher to send current status of the robot of what it is doing
        self.status = rospy.Publisher('robot1/current_status', String, queue_size=10)
        # self.send_velocity_info = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # rospy.Timer(rospy.Duration(.1), self.keep_sending_status)

    def main_code(self):
        """main code to operate the robot for it to pick up the needed items
        """
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            current_state=self.order_info.current_state()
            rospy.logdebug("in main while")
            if current_state=="picking_up_items":
                rospy.loginfo("picking up items")
                desired_goal=self.order_info.current_goal()
                rospy.loginfo(desired_goal)
                #------------------move_to_goal------------------------
                
                
                self.order_info.new_point()
                #
            elif current_state=="moving_to_drop_off":
                desired_goal=self.order_info.current_goal()
                #------------------move_to_goal------------------------
                self.order_info.new_point()
            elif current_state=="moving_back_home":
                desired_goal=self.order_info.current_goal()
                #------------------move_to_goal------------------------
                self.order_info.new_point()
            rospy.sleep(.1)
########################################################################################
if __name__ == '__main__':
    rospy.init_node("robot_main", log_level=rospy.INFO)
    item=ros_class()
    rospy.Timer(rospy.Duration(.1), item.keep_sending_status)
    item.main_code()



