#!/usr/bin/env python3
 
from __future__ import print_function

import rospy
from motion_controller.srv import path, pathResponse
from motion_controller.msg import point
import algo_functions.algorithms as algorithms
import algo_functions.algorithm_publisher_subscriber as algo_functions
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelStateRequest, GetModelState,GetModelPropertiesResponse
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import GridCells
import numpy 
import cv2
import sys
from scipy import ndimage

class algo_service:
    def convert_pnt_real_wld(self,pnt):
        global start_location, given_coor
        new_pnt=[None,None]
        new_pnt[0]=int(pnt[0]/.05)
        new_pnt[1]=int(pnt[1]/.05)
    
        # print(start_location)
        rospy.loginfo(f"coordinates:{new_pnt}")
        return new_pnt
    
    def handle_add_two_ints(self,req):
        start_pnt=self.convert_pnt_real_wld(req.start_point)
        end_pnt=self.convert_pnt_real_wld(req.end_point)
        print(end_pnt)
        rospy.logdebug(f"Received starting points: {start_pnt}, ending point: {end_pnt}")
        # print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))

        start_end_pnt=[]
        highlight=Point()
        highlight.x=(start_pnt[0])*.05+.025         # translate from grid space coordinate to world space coordinate
        highlight.y=(start_pnt[1])*.05+.025
        highlight.z=0
        start_end_pnt.append(highlight)

        highlight=Point()
        highlight.x=(end_pnt[0])*.05+.025         # translate from grid space coordinate to world space coordinate
        highlight.y=(end_pnt[1])*.05+.025
        highlight.z=0
        start_end_pnt.append(highlight)

        grid_cells_msg = GridCells()
        grid_cells_msg.cell_width = self.grid[3]
        grid_cells_msg.cell_height = self.grid[3]
        grid_cells_msg.cells = start_end_pnt
        grid_cells_msg.header.frame_id = "map"
        self.start_end_pntPub.publish(grid_cells_msg)

        rate = rospy.Rate(15)
        
       # initialize path planning object
        dimension = len(self.arr)                                                 # occupancy grid is square
        # dimension=1984
        rand_area = [1,2]                                                              # for RRT later
        step_size = 10
        robot_planner = algorithms.Algorithms([start_pnt[0],start_pnt[1]],[end_pnt[0],end_pnt[1]],dimension,dimension,list(self.rotated_array), rand_area,step_size, self.grid)    # create path planning object
        
        # checkpoint testing
        check_order = robot_planner.checkpoint_order()
        print(f'These are the checkpoints: {check_order}')               # print the checkpoint list
      
        path_array=[]
        print('Sending A* to rviz')
        for value in check_order:
            print(value)                        # print path
            #Display points that it has visited
            highlight=Point()
            highlight.x=(value[0]-1000)*.05+.025         # translate from grid space coordinate to world space coordinate
            highlight.y=(value[1]-1000)*.05+.025
            highlight.z=0
            path_array.append(highlight)

            grid_cells_msg = GridCells()
            grid_cells_msg.cell_width = self.grid[3]
            grid_cells_msg.cell_height = self.grid[3]
            grid_cells_msg.cells = path_array
            grid_cells_msg.header.frame_id = "map"
            self.start_end_pntPub.publish(grid_cells_msg)
        
        for w in range(1,5):
            self.start_end_pntPub.publish(grid_cells_msg)
            rate.sleep()
      
        print(f'This is the checkpoint order')

        robot_planner.a_star()                                                       # call a_star method, no expected return
        # print(f"This is the A_star path: {robot_planner.path['A_star']}")                           # access A_star key to display path

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
            grid_cells_msg.cell_width = self.grid[3]
            grid_cells_msg.cell_height = self.grid[3]
            grid_cells_msg.cells = path_array
            grid_cells_msg.header.frame_id = "map"
            self.cSpacePub.publish(grid_cells_msg)
        
        for w in range(1,5):
            self.cSpacePub.publish(grid_cells_msg)
            rate.sleep()

        resp_array=[]
        for item in path_array:
            new_point=point([item.x,item.y])
            resp_array.append(new_point)
        # rsp=pathResponse([point([2,0]),point([7,0]),point([8,1]),
        #                 point([9,0]),point([11,0]),point([13,0]),
        #                 point([15,0]),point([15,-3]),point([15,-5])])
        #process path
        # rsp.path=[[1,2],[1,2]]
        resp_array.append(point(req.end_point))
        rsp=pathResponse(resp_array)
        rospy.loginfo(f"Found a path: {rsp}")
        return rsp
 
    def __init__(self):
        rospy.init_node('planner_server', log_level=rospy.DEBUG)
        self.cSpacePub = rospy.Publisher('/resulting_path', GridCells, queue_size=10)
        self.start_end_pntPub = rospy.Publisher('/start_end_pnt', GridCells, queue_size=10)

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
        convert_values=ndimage.binary_dilation(convert_values, structure=struct1,iterations=5).astype(convert_values.dtype)
        # rotated_array = cv2.dilate(rotated_array, kernel, iterations=1)  

        copy_original_map=rotated_array
        for iy, ix in numpy.ndindex(convert_values.shape):
            if convert_values[iy,ix]==1 and not copy_original_map[iy,ix]==1:
                rotated_array[iy,ix]=1

        self.rotated_array=rotated_array
        self.grid=grid
        self.arr=arr

        self.s = rospy.Service('path_planner', path, self.handle_add_two_ints)
if __name__ == "__main__":
    service=algo_service()
    rospy.loginfo("planner service ready")
    rospy.spin()