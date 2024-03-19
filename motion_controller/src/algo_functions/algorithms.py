# Contains all algorithms that will be used
import time
import heapq  # for priority queues
#from RBE550_side_walk_delivery.motion_controller.src.algo_functions.algorithm_publisher_subscriber import *
import algo_functions.algorithm_publisher_subscriber as algo_functions
import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from collections import deque
import queue
# obstacle interpreter functions


class Algorithms:
    def __init__(self, start, goal, width, height, grid, rand_area, expand_dis=0.5, goal_sample_rate=20, max_iter=2000):
        self.path = {}  # path that will be appended to, dictionary of {'algorithm type': path}
        self.start = start
        self.start[0]+=1000
        self.start[1]+=1000
        self.goal = goal
        self.goal[0]+=1000
        self.goal[1]+=1000
        self.occupancy_grid = grid              #  grid object contains 3 indices, 1st is 1D array, 2nd is
        self.accuracy_radius = 20       # specify how close to the target the planner should get       
        
        # RRT search properties
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.node_list = [self.start]

        # print(len(self.occupancy_grid))
        self.grid_width = width
        self.grid_height = height

        # checkpoint parameters
        self.checkpoint_list = queue.LifoQueue()              # manual editing for now
        test_point = [1241,1015]
        self.building_checkpoints =[]       # create list of building checkpoints
        self.inside_checkpoints = []        # create list of indoors checkpoints
       
        self.checkpoint_list.put(self.goal) # first push goal onto stack
        # self.checkpoint_list.put(test_point)  # stack other checkpoints on top of goal     

    def map_callback(self, msg):
        self.map_data = msg

    def euclid_distance(self, current_position,next_position):
        distance = ((current_position[0] - next_position[0]) ** 2 + (current_position[1] - next_position[1]) ** 2) ** .5
        return distance

    def find_neighbors(self, position):
        occupancy_values = {}
        # neighbor spacing, increase to increase algorithm step size
        step_size = 5
        # this is currently (row,column) notation, not x and y
        neighbors = [[position[0] + step_size, position[1]],
                     [position[0] - step_size, position[1]],
                     [position[0] + step_size, position[1] + step_size],
                     [position[0] + step_size, position[1] - step_size],
                     [position[0] - step_size, position[1] + step_size],
                     [position[0] - step_size, position[1] - step_size],
                     [position[0], position[1] + step_size],
                     [position[0], position[1] - step_size]
                     ]
        rospy.logdebug(print(f'here are the neighbors: {neighbors}'))
        return neighbors


    def a_star(self):
        # self.path['A_star'] = [self.start, self.goal]  # store the results of A star search into the dictionary
        # return
        priority_queue = [(0, self.start)]
        visited = []
        path_tracker = []
        start_time = time.time()
        count = 0                                       # iteration counter for limit setting
        checkpoint = self.checkpoint_list.get()           # set first checkpoint
        while priority_queue:
            rospy.logdebug(f"The current priority queue {priority_queue}")                              # is you set ROS to debug, then it prints
            current_distance, current_coord = heapq.heappop(priority_queue)
            path_tracker.append(current_coord)
            print(f'This is current position: {current_coord}')
            if current_coord in visited:
                continue
            # visited.add(current_coord)
            visited.append(current_coord)
            # if current_coord == self.goal:
            #     print('A star goal reached')
            #     break

            if self.euclid_distance(current_coord,self.goal) <= 10:            # less exact target for goal
                print('A star goal reached')
                break
            neighbors = self.find_neighbors(current_coord)
            

            if self.euclid_distance(current_coord,checkpoint) <= 10:       # if planner reaches checkpoint, pop next checkpoint 
                checkpoint = self.checkpoint_list.get()

            for n in neighbors:
                # maybe include boundary check?
                print(self.occupancy_grid[n[0]][n[1]])        # to check occupancy values
                if self.occupancy_grid[n[0]][n[1]] == 0:                                             # 1 is obstacle, 0 is free space, -1 undefined
                    # cost = self.euclid_distance(n,self.goal) + self.euclid_distance(current_coord,n)     # astar equation with goal consideration
                    cost = self.euclid_distance(n,checkpoint) + self.euclid_distance(current_coord,n)      # a* equation with checkpoints
                    heapq.heappush(priority_queue, (cost, n))
            count = +1
        end_time = time.time()
        total_time = end_time - start_time
        print(f'A star took: {total_time} seconds')
        rospy.logdebug(f'Is stack empty? {self.checkpoint_list.empty()}')
        self.path['A_star'] = path_tracker  # store the results of A star search into the dictionary
        # return path_tracker


    def checkpoint_finder(self):
        # we have two options, either create a script that will divide the map into different square regions, the middle will serve as checkpoint
        # or we just manually assign the check points

        # for loop row          # nested for loop to identify a square region in map
            # for loop column
            # find center, push to self.checkpoint_list       

        # we will have two hiearchies, first is building to building nodes, after that the 2nd stack will be inside building nodes
        
            

            # orienation: east-west is parallel to the road, north is side with buildings (using xy coordinates)
            # alley way:
            # southwest corner center building 
            # coord = [1241 1015]      # these are coordinates from ROS pose messages
            # southeast corner center building 
            # coord = [721, 994]         

            # building-building nodes
            # warehouse door facing the street
            # coord = [1450,989]
            # center building outside door facing the street
            # coord = [995,989]
            # center building outside western door
            # coord = [1264,897]
            # center building outside eastern door
            # coord = [750,714]
            # rightmost building, outside door facing road
            # coord = [581,989]
            self.building_checkpoints = [[1450,989],
                                         [995,989],
                                         [1264,897],
                                         [750,714],
                                         [581,989]]
                                         

            # inside building nodes
            # center building inside hallway of door facing street
            # coord = [993,935]
            # center building western door inside hallway
            # coord = [1224, 897]

        pass

    def checkpoint_order(self):
        '''
        implement a mini-a* search across the building nodes to determine searching order with step size of 5 or something
        return checkpoint stack which will be this: [building_checkpoints, indoor_checkpoints]
        '''
        # for loop
            # priority queue heappush checkpoints based on euclidean distance to goal
        # return a queue for astar function to use


        
        pass
    def rrt_search(self):

        pass

    def hybrid_aStar(self):  # ?
        pass


# Node for RRT
    # in progress, not for a_star
class Node:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
