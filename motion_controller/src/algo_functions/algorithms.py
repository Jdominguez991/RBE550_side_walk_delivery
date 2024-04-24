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
    def __init__(self, start, goal, width, height, grid, rand_area, step_size,expand_dis=0.5, goal_sample_rate=20, max_iter=2000):
        self.path = {}  # path that will be appended to, dictionary of {'algorithm type': path}
        self.start = start
        self.start[0]+=1000
        self.start[1]+=1000
        self.goal = goal
        self.goal[0]+=1000
        self.goal[1]+=1000
        self.occupancy_grid = grid              #  grid object contains 3 indices, 1st is 1D array, 2nd is
        self.accuracy_radius = 20       # specify how close to the target the planner should get       
        self.step_size = step_size
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
        # test_point = [1241,1015]
        self.building_checkpoint_graph =[]       # create list of building checkpoints
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
        step_size = self.step_size
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
            #rospy.logdebug(f"The current priority queue {priority_queue}")                              # is you set ROS to debug, then it prints
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
            
            # condition to find shorter route without using queues
            # first condition checks if start and goal have same node, if so then remove all from queue and just find goal
            queue_list = list(self.checkpoint_list.queue)
            if len(queue_list) >=2:
                if queue_list[0] == queue_list[1]:
                    first = self.checkpoint_list.get()
                    second = self.checkpoint_list.get()
                    checkpoint = self.goal

            elif self.euclid_distance(current_coord,checkpoint) <= 10:       # if planner reaches checkpoint, pop next checkpoint 
                checkpoint = self.checkpoint_list.get()
            


            for n in neighbors:
                # maybe include boundary check?
                print(n)
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
            # warehouse building door facing the street
            # coord = [1450,989]
            # center building outside door facing the street
            # coord = [995,989]
            # center building outside western door
            # coord = [1264,897]
            # center building outside eastern door
            # coord = [750,714]
            # rightmost building, outside door facing road
            # coord = [581,989]
            building_checkpoints = [[1450,989],
                                         [995,989],
                                         [1264,897],
                                         [750,714],
                                         [581,989]]
            # this graph represents all building nodes and corner nodes, may be expanded later
            self.building_checkpoint_graph = {(1450,989): [(1241,1015)],
                                              (995,989): [(1241,1015),(721,994)],
                                              (1264,897): [(1241,1015)],
                                              (750,714): [(721,994)],
                                              (581,989): [(721,994)],
                                              (1241,1015): [(1450,989),(995,989),(1264,897)],
                                              (721,994): [(750,714),(581,989)]}
                              
            # maybe i need to represent nodes as letters or something easier?
            
            # self.building_checkpoint_graph = {(1450,989): [(1241,1015)],
            #                                   (995,989): [(1241,1015),(721,994)],
            #                                   (1264,897): [(1241,1015)],
            #                                   (750,714): [(721,994)],
            #                                   (581,989): [(721,994)],
            #                                   (1241,1015): [(1450,989),(995,989),(1264,897)],
            #                                   (721,994): [(750,714),(581,989)]}


            # inside building nodes
            # center building inside hallway of door facing street
            # coord = [993,935]
            # center building western door inside hallway
            # coord = [1224, 897]

    def min_key_finder(self,dict):
        min_distance = min(dict.values())
        for key,value in dict.items():
            if value == min_distance:
                return key

    def checkpoint_order(self):             
        '''
        implement a mini bfs search across the building nodes to determine searching order with step size of 5 or something
        return checkpoint stack which will be this: [building_checkpoints, indoor_checkpoints]

        actually maybe it would be better to just have one big graph
        '''
        self.checkpoint_finder()
        # iterate through all building nodes to find the distance from start node
        start_node_costs = {}
        for key,coordinates in self.building_checkpoint_graph.items():
            for c in coordinates:
                start_node_costs[c] = self.euclid_distance(self.start,c)
        # find nearest checkpoint
        start_node = self.min_key_finder(start_node_costs)
        rospy.logdebug(f'This is the start node: {start_node}')
        list_of_checkpoints = []
        current_node = start_node

        # find the goal checkpoint node
        goal_node_costs = {}
        for key,coordinates in self.building_checkpoint_graph.items():
            for c in coordinates:
                goal_node_costs[c] = self.euclid_distance(self.goal,c)
        end_node = self.min_key_finder(goal_node_costs)
        rospy.logdebug(f'This is the end node: {end_node}')

        search_queue = queue.Queue()
        search_queue.put(start_node)
        visited = set()
        path=[]
        new_path = list(path)  # Create a new path
        # BFS search
        while search_queue:
            
            path.append(search_queue.get())
            node = path[-1]
            node_tuple = tuple(node)
            if node_tuple not in visited:
            
                visited.add(node_tuple)

                # Check if this node is the end node
                if node_tuple == end_node:
                    return path  # Return the path if it reaches the end

                # Enqueue neighbors of the current node
                for neighbor in self.building_checkpoint_graph.get(node_tuple, ()):
                    new_path.append(neighbor)
                    search_queue.put(neighbor)
        
        rospy.logfatal("jumped out of while loop error")

        # stack nodes into checkpoint FIFO queue
            
        # list_of_checkpoints.sort(None,True)
        # for checkpt in list_of_checkpoints:
        #     self.checkpoint_list.put(checkpt)

        # now implmement for the 
        
       
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