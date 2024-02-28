# Contains all algorithms that will be used
import time
import heapq  # for priority queues
from algorithm_publisher_subscriber import *
import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped


# obstacle interpreter functions
def read_pgm(filename):
    with open(filename, 'rb') as f:
        # Read and parse PGM header
        magic = f.readline()
        if magic != b'P5\n':
            raise ValueError("Invalid PGM file format")

        width, height = map(int, f.readline().split())
        maxval = int(f.readline())

        # Read pixel values
        data = bytearray(f.read())

        return width, height, maxval, data


def interpret_obstacles(filename, threshold=128):
    width, height, maxval, data = read_pgm(filename)
    # Interpret pixel values as occupancy information
    occupancy_grid = [[0 for _ in range(width)] for _ in range(height)]
    for y in range(height):
        for x in range(width):
            pixel_value = data[y * width + x]
            if pixel_value > threshold:
                occupancy_grid[y][x] = 100  # Obstacle
            else:
                occupancy_grid[y][x] = 0  # Free space

    return occupancy_grid


# Example usage:


class Algorithms:
    def __init__(self, start, goal, width, height, grid, rand_area, expand_dis=0.5, goal_sample_rate=20, max_iter=2000):
        self.path = {}  # path that will be appended to, dictionary of {'algorithm type': path}
        self.start = start
        self.goal = goal
        self.occupancy_grid = grid              #  grid object contains 3 indices, 1st is 1D array, 2nd is       
       
        # RRT search properties
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.node_list = [self.start]

        # width, height, maxval, data = read_pgm(filename)
        # print(len(self.occupancy_grid))
        self.grid_width = len(self.occupancy_grid)
        self.grid_height = self.grid_width

        # # create node for a_star
        # rospy.init_node('a_star_search', anonymous=True)
        # self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        # self.path_pub = rospy.Publisher('/path', Path, queue_size=10)  # queue size needs to increase
        # self.map_data = None

    def map_callback(self, msg):
        self.map_data = msg

    def euclid_distance(self, current_position,next_position):
        distance = ((current_position[0] - next_position[0]) ** 2 + (current_position[1] - next_position[1]) ** 2) ** .5
        return distance

    def find_neighbors(self, position):
        occupancy_values = {}
        # this is currently (row,column) notation, not x and y
        neighbors = [(position[0] + 1, position[1]),
                     (position[0] - 1, position[1]),
                     (position[0] + 1, position[1] + 1),
                     (position[0] + 1, position[1] - 1),
                     (position[0] - 1, position[1] + 1),
                     (position[0] - 1, position[1] - 1),
                     (position[0], position[1] + 1),
                     (position[0], position[1] - 1)
                     ]
        # for loop of neighbors keys assigned to occupancy values based on occupancy grid
        for neighbor in neighbors:
            row, column = neighbor
            if 0 <= row < self.grid_height and 0 <= column < self.grid_width:
                occupancy_values[neighbor] = self.occupancy_grid[row][column]

        return neighbors, occupancy_values

    def a_star(self):
        priority_queue = [(0, self.start)]
        visited = set()
        path_tracker = []
        start_time = time.time()
        count = 0
        while priority_queue:
            current_distance, current_coord = heapq.heappop(priority_queue)
            path_tracker.append(current_coord)
            if current_coord in visited:
                continue
            visited.add(current_coord)
            if current_coord == self.goal:
                print('A star goal reached')
                break
            neighbors, occupancy_dict = self.find_neighbors(current_coord)

            for n in neighbors:
                if 0 <= n[0] < self.grid_height and 0 <= n[1] < self.grid_width:
                    if occupancy_dict[n] != 100:                                                              # if grid value is not 100, then execute stack
                        cost = self.euclid_distance(n,self.goal) + self.euclid_distance(current_coord,n)      # equation is g + h, euclid plus distance to neighbor
                        heapq.heappush(priority_queue, (cost, n))
            count = +1
        end_time = time.time()
        total_time = end_time - start_time
        print(f'A star took: {total_time}')
        self.path['Astar'] = path_tracker  # store the results of A star search into the dictionary
        return path_tracker

    def rrt_search(self):

        pass

    def hybrid_aStar(self):  # ?
        pass


# Node for RRT
    # in progress
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
