#!/usr/bin/env python

import sqlite3
from struct import pack
import tkinter as tk
from tkinter import LEFT, RIGHT, ttk
import signal
from threading import Thread
import pandas as pd
# from scipy.spatial.transform import Rotation
from curses import mousemask
import math
from operator import contains
import queue
from symbol import import_as_name
from collections import deque as queue
import time
import re
from typing import final
import sys
import rospy
from scipy import ndimage
import numpy
import cv2 as cv
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool, String
from robot_operation.srv import send_order

# install tkinter sudo apt-get install python3-tk
# pip3 install numpy scipy matplotlib ipython jupyter pandas sympy nose

#sqlite db https://www.digitalocean.com/community/tutorials/how-to-use-the-sqlite3-module-in-python-3

#tkinter into https://realpython.com/python-gui-tkinter/

# cursor.execute("DROP TABLE IF EXISTS order_tbl")
# cursor.execute("DROP TABLE IF EXISTS favorite_spots_tbl")

#create db for the orders
# cursor.execute("CREATE TABLE if NOT EXISTS order_tbl (orderID TEXT, productID TEXT, pickup_pnt TEXT, end_pnt TEXT)")

# #create db for favorite spots like the pick up locations and the end points
# cursor.execute("CREATE TABLE if NOT EXISTS favorite_spots_tbl (name TEXT, location TEXT)")
db_file_name="./order.db"

index=0
robots=[None] #for each robot add another None item in the array
status_functions=[]
robot_status=[]
columns_in_db=['orderID','productID','num_items_r','num_items_g','num_items_b','robot_assigned', 'pickup_pnt', 'end_pnt']
order_data={"start_data":None, "end_data":None, "vld_start":False, "vld_end":False,"vld_text_input":False}
resolution_cell_per_meter=1
def initialize_ros_node():
    """Initialize all the parts needed to interact with ros
    """
    global status_functions
    global robot_status
    # Initialize the node and call it "path_planner"
    rospy.init_node("path_planner")
    cSpacePub = rospy.Publisher('/path_planner/cspace', GridCells, queue_size=10)
    begin_end = rospy.Publisher('/begin_end', GridCells, queue_size=10)
    # rospy.wait_for_message('move_base_simple/goal', PoseStamped)
    rospy.Subscriber('move_base_simple/goal', PoseStamped, rviz_pnt_select)

    # For assuming multiple robot will be used
    for num in range(len(robots)):
        robots[num] = [rospy.Publisher(f'/robot{num}/start', Pose, queue_size=10), rospy.Publisher(f'/robot{num}/end', Pose, queue_size=10)] #msg or type []
    status_functions=[create_status_func(i) for i in range(len(robots))]

    robot_status.append("NOT_AVAIL")
    # Make subscribers for the current status of robots
    # for num in range(len(robots)):
    #     robot_status.append(rospy.Subscriber(f'/robot{num}/current_status', String, status_functions[num]))
    
    rospy.Subscriber(f'/robot1/current_status', String, update_status)
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
def rviz_pnt_select(data):
    """Convert the rviz gaol point that a user selects into x and y position

    Args:
        data (geometry_msgs/PoseWithCovarianceStamped): msg with information for where the end position of the robt should be
    """
    x_pos=data.pose.position.x
    y_pos=data.pose.position.y
    quaternion_angle=[data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w]

    # rot = Rotation.from_quat(quaternion_angle)
    rot=euler_from_quaternion(quaternion_angle)
    rot_euler = rot.as_euler('xyz', degrees=True)

    #Replace the values on tkinter with the values that were selected from rviz 
    x_pos_entry.delete(0,tk.END)
    x_pos_entry.insert(0,int(x_pos/resolution_cell_per_meter))

    y_pos_entry.delete(0,tk.END)
    y_pos_entry.insert(0,int(y_pos/resolution_cell_per_meter))

    z_ang_entry.delete(0,tk.END)
    z_ang_entry.insert(0,round(rot_euler[2], 2))

    # rvix_x_position.config(text=f"rviz x position= {int(x_pos/resolution_cell_per_meter)}")
    # rvix_y_position.config(text=f"rviz y position= {int(y_pos/resolution_cell_per_meter)}")
    # rvix_z_angle.config(text=f"rviz z angle= {rot_euler[2]}")

def create_status_func(robot_num):
    """create a function that can be placed into a list to be called later on

    Args:
        robot_num (string): The number of robot this sub function should get used in

    Return:
        function: return function to grab robot status
    """
    def update_status(data):
        """Update the global status of the robot

        Args:
            data (str): msg with information of status of that robot
        """
        global robot_status
        robot_status[robot_num]=data.data #should be of string type
    return update_status

def update_status(data):
    """Update the global status of the robot

    Args:
        data (str): msg with information of status of that robot
    """
    global robot_status
    robot_status[0]=data.data #should be of string type
    
    
def connect():
    """Create data base file to handle orders
    """
    global db_file_name, columns_in_db
    con1 = sqlite3.connect(db_file_name)
    print(con1)
    cur1 = con1.cursor()
    cur1.execute('DROP TABLE if EXISTS order_tbl')
    # cur1.execute('DROP TABLE if EXISTS favorite_spots_tbl')
    #create db for the orders
    initial_string="CREATE TABLE if NOT EXISTS order_tbl ("
    for num, item in enumerate(columns_in_db):
        initial_string+=f"{item} TEXT"
        if(num!=len(columns_in_db)-1):
            initial_string+=", "
        else:
            initial_string+=")"
    print(initial_string)
    cur1.execute(initial_string)

    #create db for faortive spots like the pick up locations and the end points
    cur1.execute("CREATE TABLE if NOT EXISTS favorite_spots_tbl (name TEXT, location TEXT, angle TEXT)")
    # cur1.execute("INSERT INTO favorite_spots_tbl VALUES('pickup_one','[123,-45]','0')")
    # cur1.execute("INSERT INTO favorite_spots_tbl VALUES('pickup_one2','[1243,-45]','0')")
    con1.commit()
    con1.close()

def view_orders():
    """Update all the data o the order tree to display what is currently in database
    """
    global db_file_name
    rospy.logdebug("view funct")
    con1 = sqlite3.connect(db_file_name)
    cur1 = con1.cursor()
    cur1.execute("SELECT * FROM order_tbl")
    rows = cur1.fetchall()  
    tree.delete(*tree.get_children()) #Clear data in tree to only display data once not repetative
    for row in rows:
        tree.insert("", tk.END, values=row)        
    con1.close()

def view_locations():
    """Refresh the display to display the locations that are saved
    """
    global db_file_name
    rospy.logdebug("view funct_locations")
    con1 = sqlite3.connect(db_file_name)
    cur1 = con1.cursor()
    cur1.execute("SELECT * FROM favorite_spots_tbl")
    rows = cur1.fetchall()  
    rospy.logdebug(rows)
    location_tree.delete(*location_tree.get_children())
    for row in rows:
        location_tree.insert("", tk.END, values=row)        
    con1.close()

def upload_data():
    """upload the order info into the order db
    """
    global index, db_file_name, order_data
    con1 = sqlite3.connect(db_file_name)
    rospy.logdebug("uploading data")
    cur1 = con1.cursor()
    cur1.execute(f"INSERT INTO order_tbl VALUES ('{name_item_entry.get()}#{index}', '{name_item_entry.get()}#{index}', '{entry1.get()}','{entry2.get()}','{entry3.get()}', 'NOT ASSIGNED', '({order_data['start_data'][1]},{order_data['start_data'][2]})','({order_data['end_data'][1]},{order_data['end_data'][2]})')")
    index+=1
    con1.commit()
    con1.close()
    view_orders()

def select_start():
    """Select where the starting location is
    """
    global location_tree, order_data
    # loc_value = tree.set(a, column="loc")

        # Get the selected iid
    selected_item = location_tree.focus()

    # Get a dictionary of details about the iid
    item_details = location_tree.item(selected_item)

    # The row's displayed text will be in the 'values' key.
    array_data=item_details.get("values")

    if order_data["end_data"] == array_data:
        pickup_label.config(text=f"Pick spot position= PICKUP POINT CANNOT BE SAME AS DROP-OFF")
        order_data["vld_start"]=False
        return
    else:
        order_data["start_data"]=array_data
        order_data["vld_start"]=True
    
    if array_data:
        pickup_label.config(text=f"Pick spot position= {array_data[1]}")
    else:
        print("Please Select start pnt")

def select_dropoff():
    """Select drop off location
    """
    global location_tree, order_data
    # loc_value = tree.set(a, column="loc")

        # Get the selected iid
    selected_item = location_tree.focus()

    # Get a dictionary of details about the iid
    item_details = location_tree.item(selected_item)

    # The row's displayed text will be in the 'values' key.
    array_data=item_details.get("values")
    if order_data["start_data"] == array_data:
        drop_off_label.config(text=f"Drop off position= DROP-OFF POINT CANNOT BE SAME AS PICKUP")
        order_data["vld_end"]=False
        return
    else:
        order_data["end_data"]=array_data
        order_data["vld_end"]=True
    
    if array_data:
        drop_off_label.config(text=f"Drop off position= {array_data[1]}")
    else:
        print("Please Select end pnt")

def clear_pickup_drop_off_pnt():
    """Clear information for pick up and drop off points
    """
    pickup_label.config(text=f"Pick spot position= ")
    drop_off_label.config(text=f"Drop off position= ")

    order_data["end_data"]=None
    order_data["start_data"]=None
    order_data["vld_end"]=False
    order_data["vld_start"]=False
def add_pnt():
    """Add a point selected on rviz into favorite points list
    """
    con1 = sqlite3.connect(db_file_name)
    if(con1.execute(f"SELECT name FROM favorite_spots_tbl WHERE name NOT IN ('{pos_name_entry.get()}');")):
        con1.execute(f"INSERT INTO favorite_spots_tbl VALUES('{pos_name_entry.get()}','[{x_pos_entry.get()},{y_pos_entry.get()}]','{z_ang_entry.get()}')")
        con1.commit()
        con1.close()
        view_locations()
    else:
        con1.close()
        rospy.logwarn("name already exists")
def delete_item():
    """Delete a point in favorite spot tbl
    """
    global location_tree
    # loc_value = tree.set(a, column="loc")

        # Get the selected iid
    selected_item = location_tree.focus()

    # Get a dictionary of details about the iid
    item_details = location_tree.item(selected_item)

    # The row's displayed text will be in the 'values' key.
    array_data=item_details.get("values")
    
    con1 = sqlite3.connect(db_file_name)
    rospy.logdebug("uploading data")
    cur1 = con1.cursor()
    cur1.execute(f"DELETE FROM favorite_spots_tbl WHERE name = '{array_data[0]}';")
    con1.commit()
    con1.close()
    view_locations()

def send_order_data(num_red, num_blue, num_green, end_pnt, robot_num):
    split_location=end_pnt.split(',')

    item=re.findall(r"\d+", end_pnt)
    # item=[int(i) for i in end_pnt.split(',') if i.isdigit()]
    rospy.loginfo(num_red)

    rospy.loginfo("num red")

    rospy.wait_for_service(f'robot{robot_num+1}/receive_order')
    try:
        add_two_ints = rospy.ServiceProxy(f'robot{robot_num+1}/receive_order', send_order)
        resp1 = add_two_ints(int(num_red), int(num_green), int(num_blue), f'[{item[0]},{item[1]}]')
        if not resp1.received:
            rospy.logerr("service not received")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)
    sys.exit(0)
def assign_orders():
    """Assign a robot an order 
    """
    global columns_in_db, robot_status
    find_robot_index=columns_in_db.index("robot_assigned")
    rospy.loginfo(robot_status)
    while not rospy.is_shutdown():
        # rospy.loginfo("in while")
        con1 = sqlite3.connect(db_file_name)
        cur1 = con1.cursor()
        cur1.execute("SELECT * FROM order_tbl")
        rows = cur1.fetchall()  
        cur1.close()
        for robot_num, status in enumerate(robot_status):
            rospy.logdebug(f"looking robot, current stat: {status.upper()}")
            if status.upper()=="WAITING_ORDER":
                rospy.loginfo("found bot")
                if(len(rows)>0):
                    for row in rows:
                        if (row[find_robot_index]=="NOT ASSIGNED"):
                            rospy.loginfo(row[columns_in_db.index("pickup_pnt")])
                            send_order_data(row[columns_in_db.index("num_items_r")],row[columns_in_db.index("num_items_b")],row[columns_in_db.index("num_items_g")],row[columns_in_db.index("end_pnt")], robot_num)
        rospy.sleep(.1)

# def tkinter_window():
#     window.mainloop()

# Create order db
# connect()

#Register node with ros
initialize_ros_node() 

#main tkinter window
window = tk.Tk()

#frame with pick up from off info
position_frame=tk.Frame(window)

#Selecting point frame
rviz_select_frame=tk.Frame(window)

#Set up all items for ordering item
order_name = tk.Label(window, text = "Current orders")
order_name.pack()

#Current orders tree view orders in process
tree = ttk.Treeview(window, column=columns_in_db, show='headings')
for number, item in enumerate(columns_in_db):
    tree.column(f"#{number+1}", anchor=tk.CENTER)
    tree.heading(f"#{number+1}", text=item)

#Favorite locations tree
tree.pack()
favorite_locations = tk.Label(window, text = "Current orders")
favorite_locations.pack()
location_tree = ttk.Treeview(window, column=("c1", "c2","c3"), show='headings')
location_tree.column("#1", anchor=tk.CENTER)
location_tree.heading("#1", text="Name")
location_tree.column("#2", anchor=tk.CENTER)
location_tree.heading("#2", text="Location")
location_tree.column("#3", anchor=tk.CENTER)
location_tree.heading("#3", text="Location")
location_tree.pack()
view_locations()

#Clear favorite point
name = tk.Label(window, text = "Name")
button1 = tk.Button(text="Display data", command=view_orders)
button1.pack(pady=10)
button4 = tk.Button(window, text="delete_pnt", command=delete_item)
button4.pack(pady=10)

#Entering order info
name_item = tk.Label(window, text = "Name")
name_item_entry=tk.Entry(window)
name_item.pack()
name_item_entry.pack()

enter_quantity=tk.Frame(window)
name1 = tk.Label(enter_quantity, text = "Num Item R")
entry1=tk.Entry(enter_quantity)
name1.grid(row = 0, column = 0, pady = 2)
entry1.grid(row = 1, column = 0, pady = 2)


name2 = tk.Label(enter_quantity, text = "Num Item G")
entry2=tk.Entry(enter_quantity)
name2.grid(row = 0, column = 1, pady = 2)
entry2.grid(row = 1, column = 1, pady = 2)


name3 = tk.Label(enter_quantity, text = "Num Item B")
entry3=tk.Entry(enter_quantity)
name3.grid(row = 0, column = 2, pady = 2)
entry3.grid(row = 1, column = 2, pady = 2)
enter_quantity.pack()

pickup_label=tk.Label(position_frame, text = "Pick spot position= ")
pickup_label.pack()

drop_off_label=tk.Label(position_frame, text = "Drop off position= ")
drop_off_label.pack()

start_pnt_btn = tk.Button(position_frame, text="Select_start_pnt", command=select_start)
start_pnt_btn.pack(pady=10, side=LEFT)

clear_data_btn = tk.Button(position_frame, text="Clear Data Pnt's", command=clear_pickup_drop_off_pnt)
clear_data_btn.pack(pady=10, side=RIGHT)

drop_off_btn = tk.Button(position_frame, text="Select_dropoff_pnt", command=select_dropoff)
drop_off_btn.pack(pady=10, side=RIGHT)

position_frame.pack()
button2 = tk.Button(text="upload Data", command=upload_data)
button2.pack(pady=10)

# Buttons related to selecting a point from rviz
rviz_select_frame.pack()

name_frame=tk.Frame(rviz_select_frame)
ask_pos_name=tk.Label(name_frame, text = "location name= ")
ask_pos_name.pack(side=LEFT)
pos_name_entry=tk.Entry(name_frame)
pos_name_entry.pack(side=RIGHT)
name_frame.pack()

x_rviz_frame=tk.Frame(rviz_select_frame)
rviz_x_position=tk.Label(x_rviz_frame, text = "rviz x position= ")
rviz_x_position.pack(side=LEFT)
x_pos_entry=tk.Entry(x_rviz_frame)
x_pos_entry.pack(side=RIGHT)
x_rviz_frame.pack()
# rvix_x_position.pack()

y_rviz_frame=tk.Frame(rviz_select_frame)
rviz_y_position=tk.Label(y_rviz_frame, text = "rviz y position= ")
rviz_y_position.pack(side=LEFT)
y_pos_entry=tk.Entry(y_rviz_frame)
y_pos_entry.pack(side=RIGHT)
y_rviz_frame.pack()
# rvix_y_position.pack()

z_rviz_frame=tk.Frame(rviz_select_frame)
rviz_z_angle=tk.Label(z_rviz_frame, text = "rviz z angle= ")
rviz_z_angle.pack(side=LEFT)
z_ang_entry=tk.Entry(z_rviz_frame)
z_ang_entry.pack(side=RIGHT)
z_rviz_frame.pack()
# rvix_z_angle.pack()

add_pnt_btn= tk.Button(text="add pnt to favorites", command=add_pnt)
add_pnt_btn.pack(pady=10)

#Thread to assign orders to robots
thread = Thread(target = assign_orders)
thread.start() # This code will execute in parallel to the current code

window.mainloop()
thread.join
# window.mainloop()