from frontier import Frontier
import sys
 

class Maze:
    '''
    A maze that use a list of adjacency neighbours
    Each node is represented by an integer
    '''
    
    
    def __init__(self, size, list_node_neighbours=None):
        self.size = size
        self.adj_list = {} # dict of node -> neighbours
        
        if list_node_neighbours:
            for elem in list_node_neighbours:
                self.addNeighbours(elem[0], elem[1])
    
    
    @classmethod
    def fromTestFile(cls, file_path):
        list_node_neighbours = [] # arg to pass into Maze constructor
        file = open(file_path, 'r') 
        
        n = int(file.readline())
        for i in range(n * n):
            # convert list of str to list of int
            neighbours = [int(elem) for elem in file.readline().split(' ')]
            list_node_neighbours.append((i, neighbours))
            
        goal_node = int(file.readline())
        
        file.close()
            
        return cls(n, list_node_neighbours), goal_node
    
    
    def manhattanDistance(self, start, end):
        i, j = divmod(start, self.size)
        i_goal, j_goal = divmod(end, self.size)
        
        i_delta = abs(i - i_goal)
        j_delta = abs(j - j_goal)
    
        return i_delta + j_delta
        
    
    def addNeighbours(self, node, neighbours):
        self.adj_list[node] = neighbours
        
        
    def breadthFirstSearch(self, start, goal):
        from queue import Queue
        
        if start == goal:
            return [], [], 0
        
        frontier = Queue()
        frontier.put([start])
        explored_list = []
        time_to_escape = 0
    
        while True:
            if frontier.empty():
                break
            
            # pop from frontier
            cur_path = frontier.get()
            cur_node = cur_path[-1]
            
            # check lazy delete
            if cur_node in explored_list:
                continue
            
            # add current node to explored list
            explored_list.append(cur_node)
            
            # Get all neighbours of current node in asc order
            neighbours = self.adj_list[cur_node]
            neighbours.sort()
    
            for neighbour in neighbours:
                if neighbour not in explored_list:
                    # new path to go to a neighbour
                    path_to_neighbour = cur_path.copy()
                    path_to_neighbour.append(neighbour)
                    
                    time_to_escape += 1
                    
                    # test goal condition
                    if neighbour == goal:           
                        return path_to_neighbour, explored_list, time_to_escape
                                        
                    # use lazy delete 
                    # -> no need to check if path is already in frontier or not
                    frontier.put(path_to_neighbour)
                
        return None, explored_list, time_to_escape
    
    
    def uniformCostSearch(self, start, goal):
        if start == goal:
            return [], [], 0
        
         # heap of tuple(path_cost, path)
        frontier = Frontier()
        frontier.push((0, [start]))
        explored_list = []
        time_to_escape = 0
    
        while True:
            if not frontier:
                break
            
            # pop min node from the frontier
            cur_path_cost, cur_path = frontier.pop()
            cur_node = cur_path[-1]
            
            # check lazy delete
            if cur_node in explored_list:
                continue
            
            # test goal condition
            if cur_node == goal:                
                return cur_path, explored_list, time_to_escape
            
            # add current node to explored list
            explored_list.append(cur_node)
            
            # Get all neighbours of current node in asc order
            neighbours = self.adj_list[cur_node]
            neighbours.sort()
    
            for neighbour in neighbours:                
                if neighbour not in explored_list:
                    # new path to go to a neighbour
                    path_to_neighbour = cur_path.copy()
                    path_to_neighbour.append(neighbour)
    
                    # path cost increase by 1 to go to neighbour
                    path_to_neighbour_cost = cur_path_cost + 1
                    
                    time_to_escape += 1
    
                    # if neighbour already in frontier or not
                    # -> use lazy delete
                    frontier.push((path_to_neighbour_cost, path_to_neighbour))
                
        return None, explored_list, time_to_escape
    
    
    def depthFirstSearch(self, start, goal):
        from queue import LifoQueue # basically a stack
        
        if start == goal:
            return [], [], 0
        
        frontier = LifoQueue()
        frontier.put([start])
        explored_list = []
        time_to_escape = 0
    
        while True:
            if frontier.empty():
                break
            
            # pop from frontier
            cur_path = frontier.get()
            cur_node = cur_path[-1]
            
            # check lazy delete
            if cur_node in explored_list:
                continue
            
            # add current node to explored list
            explored_list.append(cur_node)
            
            # Get all neighbours of current node in des order 
            # to push to stack in asc order
            neighbours = self.adj_list[cur_node]
            neighbours.sort(reverse=True)
    
            for neighbour in neighbours:
                if neighbour not in explored_list:
                    # new path to go to a neighbour
                    path_to_neighbour = cur_path.copy()
                    path_to_neighbour.append(neighbour)
    
                    time_to_escape += 1
                    
                    # test goal condition
                    if neighbour == goal:           
                        return path_to_neighbour, explored_list, time_to_escape
                    
                    # use lazy delete 
                    # -> no need to check if path is already in frontier or not
                    frontier.put(path_to_neighbour)
                
        return None, explored_list, time_to_escape
    
    
    def depthLimitedSearch(self, start, goal, limit):
        from queue import LifoQueue # basically a stack

        if start == goal:
            return [], [], 0
        
        # each element is a tuple of (level, path)
        # level is the level of that path in the graph
        frontier = LifoQueue()
        frontier.put((1, [start]))
        explored_list = []
        time_to_escape = 0
    
        while True:
            if frontier.empty():
                break
            
            # pop from frontier
            cur_level, cur_path = frontier.get()
            cur_node = cur_path[-1]
            
            # check lazy delete
            if (cur_node in explored_list) or (cur_level > limit):
                continue
            
            # add current node to explored list
            explored_list.append(cur_node)
            
            # Get all neighbours of current node in des order 
            # to push to stack in asc order
            neighbours = self.adj_list[cur_node]
            neighbours.sort(reverse=True)
    
            for neighbour in neighbours:
                if neighbour not in explored_list:
                    # new path to go to a neighbour
                    path_to_neighbour = cur_path.copy()
                    path_to_neighbour.append(neighbour)
                
                    neighbour_level = cur_level + 1
                
                    time_to_escape += 1
                    
                    # test goal condition
                    if neighbour == goal:           
                        return path_to_neighbour, explored_list, time_to_escape
                    
                    # use lazy delete 
                    # -> no need to check if path is already in frontier or not
                    frontier.put((neighbour_level, path_to_neighbour))
                
        return None, explored_list, time_to_escape
    
    
    def iterativeDeepeningSearch(self, start, goal):
        if start == goal:
            return [], [[]], 0
        
        final_explored_list = []
        final_time_to_escape = 0
        
        for i in range(1, sys.maxsize):
            path, explored_list, time_to_escape = self.depthLimitedSearch(start, goal, i)
            
            final_explored_list.append(explored_list)
            final_time_to_escape += time_to_escape
            
            if path:
                return path, final_explored_list, final_time_to_escape
        
        return None, final_explored_list, time_to_escape
    
    
    def greedyBestFirstSearch(self, start, goal):
        if start == goal:
            return [], [], 0
        
         # heap of tuple(path_heuristic, path)
        frontier = Frontier()
        frontier.push((self.manhattanDistance(start, goal), [start]))
        explored_list = []
        time_to_escape = 0
    
        while True:
            if not frontier:
                break
            
            # pop min node from the frontier
            cur_path_cost, cur_path = frontier.pop()
            cur_node = cur_path[-1]
            
            # check lazy delete
            if cur_node in explored_list:
                continue
            
            # add current node to explored list
            explored_list.append(cur_node)
            
            # Get all neighbours of current node in asc order
            neighbours = self.adj_list[cur_node]
            neighbours.sort()
    
            for neighbour in neighbours:
                if neighbour not in explored_list:
                    # new path to go to a neighbour
                    path_to_neighbour = cur_path.copy()
                    path_to_neighbour.append(neighbour)
    
                    # heuristic from this neighbour to the goal
                    path_to_neighbour_heuristic = self.manhattanDistance(neighbour, goal)
    
                    time_to_escape += 1
                    
                    # test goal condition
                    if neighbour == goal:                
                        return path_to_neighbour, explored_list, time_to_escape
                    
                    # whether neighbour already in frontier or not
                    # -> insert anyway
                    # -> use lazy delete
                    frontier.push((path_to_neighbour_heuristic, path_to_neighbour))
                
        return None, explored_list, time_to_escape
    
    
    def aStarSearch(self, start, goal):
        if start == goal:
            return [], [], 0
        
        # heap of tuple(path_total_cost, path)
        # path_total_cost = path_cost + heuristic
        frontier = Frontier()
        frontier.push((self.manhattanDistance(start, goal), [start]))
        explored_list = []
        time_to_escape = 0
    
        while True:
            if not frontier:
                break
            
            # pop min node from the frontier
            cur_path_total_cost, cur_path = frontier.pop()
            cur_node = cur_path[-1]
            
            # check lazy delete
            if cur_node in explored_list:
                continue
            
            # test goal condition
            if cur_node == goal:                
                return cur_path, explored_list, time_to_escape
            
            # add current node to explored list
            explored_list.append(cur_node)
            
            # Get all neighbours of current node in asc order
            neighbours = self.adj_list[cur_node]
            neighbours.sort()
    
            for neighbour in neighbours:
                if neighbour not in explored_list:
                    # new path to go to a neighbour
                    path_to_neighbour = cur_path.copy()
                    path_to_neighbour.append(neighbour)
    
                    # calc path_total_cost (include heuristic)
                    path_to_neighbour_total_cost = cur_path_total_cost - self.manhattanDistance(cur_node, goal) + 1 + self.manhattanDistance(neighbour, goal)
                    
                    time_to_escape += 1
                    
                    # if neighbour already in frontier or not
                    # -> use lazy delete
                    frontier.push((path_to_neighbour_total_cost, path_to_neighbour))
                
        return None, explored_list, time_to_escape