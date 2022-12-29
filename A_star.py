from os import close
import numpy as np
from heapq import heappop, heappush
import matplotlib.pyplot as plt
import sys
import copy
import math as m
class Node(object):
    def __init__(self, pose):
        self.pose = np.array(pose)
        self.x = pose[0]
        self.y = pose[1]
        self.g_value = float("inf")
        self.h_value = 0
        self.f_value = float("inf")
        self.parent = None

    def __lt__(self, other):
        if self.f_value == other.f_value:
            return self.h_value < other.h_value
        return self.f_value < other.f_value

    def __eq__(self, other):
        return (self.pose == other.pose).all()

class AStar(object):
    def __init__(self, map,weight):
        self.map_p = copy.deepcopy(map)
        self.y_dim = self.map_p.shape[0]
        self.x_dim =self.map_p.shape[1]
        self.visited = {}
        self.weight = weight
        # print(f'map size ({self.x_dim}, {self.y_dim})')
        
    def getmapindex(self, x, y):
        return ((y)*(self.y_dim) + (x))
        
    def reset_map(self):
        self.visited = {}
        # return
        
    def heuristic(self, current, goal):
        dx = abs(goal.x - current.x)
        dy = abs(goal.y - current.y)
        h = (2**0.5)*min(dx,dy) + ((max(dx,dy))-(min(dx,dy)))
        # h = ((current.x-goal.x)**2 + (current.y-goal.y)**2)**0.5
        return self.weight*h
        
    def get_successor(self, node):
        successor_list = []
        x,y = node.pose
        pose_list = [[x+1, y+1], [x, y+1], [x-1, y+1], [x-1, y],
                        [x-1, y-1], [x, y-1], [x+1, y-1], [x+1, y]]

        for pose_ in pose_list:
            x_, y_ = pose_
            
            if 0 <= x_ < self.y_dim and 0 <= y_ < self.x_dim and self.map_p[x_,y_]!=2:
                successor_list.append(Node(pose_))
        
        return successor_list
    
    def calculate_path(self, node):
        path_ind = []
        path_ind.append(node.pose.tolist())
        current = node
        while current.parent is not None:
            current = current.parent
            path_ind.append(current.pose.tolist())
        path_ind.reverse()
        # print(f'path length {len(path_ind)}')
        path = list(path_ind)

        return path

    def plan(self, start_ind, goal_ind):
        start_node = Node(start_ind)
        goal_node = Node(goal_ind)
        
        start_node.h_value= self.heuristic(start_node,goal_node)
        start_node.g_value = 0
        start_node.f_value = start_node.g_value+start_node.h_value
        self.reset_map()

        open_list = []
        closed_list = []
        heappush(open_list, start_node)

        while len(open_list):
            current =heappop(open_list) # ADDING THE NODE WITH LEAST F VALUE
            closed_list = np.append(closed_list, current)

            if current == goal_node:
                return self.calculate_path(current)
            
            for successor in self.get_successor(current):
                if self.getmapindex(successor.x, successor.y) in self.visited:
                        successor = self.visited[self.getmapindex(successor.x, successor.y)]    
                        # print(successor.g_value,successor.h_value,successor.f_value)
                if successor not in closed_list:  
                    if ((successor.g_value) > current.g_value + self.map_p[successor.x,successor.y]):
                        successor.parent = current
                        successor.g_value=current.g_value + self.map_p[successor.x,successor.y] + 1 ##### CHECK !!!!!!
                        # print(successor.g_value,successor.h_value,successor.f_value)
                        successor.h_value = self.heuristic(successor,goal_node)
                        successor.f_value=successor.g_value+successor.h_value
                        self.visited[self.getmapindex(successor.x, successor.y)] = successor
                        if (successor not in open_list): 
                            heappush(open_list, successor)

        print('path not found')
        return None

    def run(self, cost_map, start_ind, goal_ind):
        if cost_map[start_ind[0], start_ind[1]] != 2 and cost_map[goal_ind[0], goal_ind[1]] != 2:
            return self.plan(start_ind, goal_ind)

        else:
            print('Goal/Start point is in Keep out zone')
