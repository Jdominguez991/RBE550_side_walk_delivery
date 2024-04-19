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

class SingletonClass(object):
  def __new__(cls):
    if not hasattr(cls, 'instance'):
      cls.instance = super(SingletonClass, cls).__new__(cls)
    return cls.instance
class acml(SingletonClass):
    def is_localized():
        rospy.loginfo("default robot always localized need to change when navigation code is made")
        return True
class main_states(StateMachine):
    waiting_acml_lock = State(initial=True)
    waiting_order = State()
    picking_up_items = State()
    moving_to_drop_off = State()
    dropped_off = State()
    moving_back_home = State()
    acml_item=acml()
    step = (
        waiting_acml_lock.to(waiting_order, cond="is_localized")
        | waiting_order.to(picking_up_items)
        | picking_up_items.to(moving_to_drop_off)
        | moving_to_drop_off.to(moving_back_home)
        | moving_back_home.to(waiting_order)
    )

    def before_step(self, event: str, source: State, target: State):
        # message = ". " + message if message else ""
        return f"Running {event} from {source.id} to {target.id}"

    def is_localized(self, event_data):
        return self.acml_item.is_localized()

    def on_enter_red(self):
        print("Don't move.")

    def on_exit_red(self):
        print("Go ahead!")

class current_order_status(SingletonClass):
    #[quantity, is_picked_up]
    object_info={"red":[0,True,[0,0]],"green":[0,True,[0,0]],"blue":[0,True,[0,0]]} #If set to true means no items from that color need to be picked up
    end_location=[0,0] #x,y
    desired_points=[]
    home_position=[0,0]
    current_robot_state=main_states()

    def reset_states(self):
        copy_info= self.object_info
        for color in copy_info.keys():
            self.object_info[color][0]=0
            self.object_info[color][1]=True

    def __init__(self):
        config_yaml=(Path(__file__).resolve().parents[2]).joinpath('order_handler/src/conf.yaml')     
        with open(config_yaml,'r') as f:
            data = yaml.load(f, Loader=SafeLoader)
            for pickup in data["pick_up_loc"]:
                if pickup["color"].upper() == "R":
                    self.object_info["red"][2]=pickup["unit_location"][:2]
                elif pickup["color"].upper() == "G":
                    self.object_info["green"][2]=pickup["unit_location"][:2]
                elif pickup["color"].upper() == "B":
                    self.object_info["blue"][2]=pickup["unit_location"][:2]
                else:
                    rospy.logerr("unknown color, please fix if statement for new color")
            self.home_position=data["robot_info"]["location"]

    def incoming_order(self,r,g,b,location):
        self.reset_states()
        if(r>0):
            self.object_info["red"][0]=[r,False]
        if(g>0):
            self.object_info["green"][0]=[g,False]
        if(b>0):
            self.object_info["blue"][0]=[b,False]

        split_location=location.split(",")
        self.end_location=[int(split_location[0]),int(split_location[1])]
        self.__process_points()
        self.current_robot_state.step()
        return True

    def __process_points(self):

        for item_num,info in enumerate(self.object_info):
            if info[0] > 0:
                if item_num==0:
                    self.desired_points.append([info[2],1])
                else:
                    self.desired_points.append([info[2],0])
        self.desired_points.append([self.end_location,1])
        self.desired_points.append([self.home_position,1])

    def new_point(self):
        self.desired_points.pop(0)
        if self.desired_points[0][1]:
            self.current_robot_state.step()
        return self.desired_points[0][0]
    
    def current_goal(self):
        return self.desired_points[0]
    
    def desired_points_list(self):
        return self.desired_points

    def current_state(self):
        self.current_robot_state.current_state.id

class ros_class(SingletonClass):
    order_info=current_order_status()

    # create a function for getting a occupancy map
    def request_map(color):
        global resolution_cell_per_meter
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                                                                                        None in case of error.
        """
        # REQUIRED CREDIT
        rospy.loginfo("Requesting the map")
        # grid = rospy.ServiceProxy('nav_msgs/GetMap', GetMap)
        rospy.wait_for_service(f'/product_spawner_{color}')
        try:
            grid = rospy.ServiceProxy(f'/product_spawner_{color}', Trigger)
            occuGrid = grid()
            return([occuGrid.map.data,occuGrid.map.info.width,occuGrid.map.info.height,occuGrid.map.info.resolution])
        except:
            rospy.loginfo("Failed")

    def received_order(self,data):
        self.order_info.incoming_order(data.r,data.g,data.b,data.location)
        return send_orderResponse(True)
    
    #Keep sending robot status for any one that wants it
    def keep_sending_status(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self.status.publish(self.order_info.current_state())
            rate.sleep()
    
    def __init__(self):
        rospy.init_node("robot_main")
        self.s = rospy.Service('robot1/receive_order', send_order, self.received_order)
        self.status = rospy.Publisher('robot1/current_status', String, queue_size=10)
        self.send_velocity_info = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.thread = Thread(target = self.keep_sending_status)
        self.thread.start() # This code will execute in parallel to the current code

    def kill_node(self):
        self.thread.join

    def send_new_velocity(self, forward_vel, angular_vel):
        msg=Twist()
        linear_vel=Vector3()
        linear_vel.x=forward_vel
        linear_vel.y=0
        linear_vel.z=0

        angular_vel=Vector3()
        angular_vel.x=0
        angular_vel.y=0
        angular_vel.z=angular_vel

        msg.linear=linear_vel
        msg.angular=angular_vel
        self.send_velocity_info.publish(msg)
    def ask_new_product(self):
        rospy.loginfo("ask new product")
        return True
    def main_code(self):
        current_state=self.order_info.current_state()
        while not rospy.is_shutdown():
            if current_state=="picking_up_items":
                desired_goal=self.order_info.current_goal()
                #------------------move_to_goal------------------------
                self.order_info.new_point()
                #
                while(True):
                    if self.ask_new_point():
                        break
                    time.sleep(1)
            elif current_state=="moving_to_drop_off":
                desired_goal=self.order_info.current_goal()
                #------------------move_to_goal------------------------
                self.order_info.new_point()
            elif current_state=="moving_back_home":
                desired_goal=self.order_info.current_goal()
                #------------------move_to_goal------------------------
                self.order_info.new_point()
########################################################################################
ros=ros_class()

