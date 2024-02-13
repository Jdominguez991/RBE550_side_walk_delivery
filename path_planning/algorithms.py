# Contains all algorithms that will be used
import time
import heapq

# TO DO: need some class to process obstacles or map information
class Algorithms:
    def __init__(self,start, goal, obstacles):
        self.path = {} # path that will be appended to, dictionary of {'algorithm type': path}
        self.start = start
        self.goal = goal
        self.obstacle_list = obstacles
        self.boundaries = []                # need to replace with map boundary dimensions or similar

    def euclid_distance(self,current_position):
        distance = ((current_position[0]-self.goal[0])**2+(current_position[1]-self.goal[1])**2)**.5
        return distance
    def find_neighbors(self,position):
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
        return neighbors

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
            neighbors = self.find_neighbors(current_coord)

            for n in neighbors:
                if 0 <= n[0] < self.boundaries[0] and 0 <= n[1] < self.boundaries[1]:
                    if n not in self.obstacle_list:
                        cost = self.euclid_distance(n) + 1              # equation is g + h, euclid plus distance to next goal
                        heapq.heappush(priority_queue, (cost, n))
            count = +1
        end_time = time.time()
        total_time = end_time - start_time
        print(f'A star took: {total_time}')
        self.path['Astar'] = path_tracker           # store the results of A star search into the dictionary
        # return path_tracker
    def rrt_search(self):
        pass

    def hybrid_aStar(self):  #?
        pass