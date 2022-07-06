# -*- coding: utf-8 -*-
# -*- coding: euc-kr -*-

import math
import numpy as np
import time
import threading
import Control

map_count = 0

next_x_previous = 0
next_y_previous = 0

from warnings import warn
import heapq

class Node:
    """
    A node class for A* Pathfinding
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position
    
    def __repr__(self):
      return f"{self.position} - g: {self.g} h: {self.h} f: {self.f}"

    # defining less than for purposes of heap queue
    def __lt__(self, other):
      return self.f < other.f
    
    # defining greater than for purposes of heap queue
    def __gt__(self, other):
      return self.f > other.f

def return_path(current_node):
    path = []
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1]  # Return reversed path


def astar(maze, start, end, allow_diagonal_movement = True):

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Heapify the open_list and Add the start node
    heapq.heapify(open_list) 
    heapq.heappush(open_list, start_node)

    # Adding a stop condition
    outer_iterations = 0
    max_iterations = (len(maze[0]) * len(maze) // 2)

    # what squares do we search
    adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0),)
    if allow_diagonal_movement:
        adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1),)

    # Loop until you find the end
    while len(open_list) > 0:
        outer_iterations += 1

        if outer_iterations > max_iterations:
          # if we hit this point return the path such as it is
          # it will not contain the destination
          warn("giving up on pathfinding too many iterations")
          return return_path(current_node)       
        
        # Get the current node
        current_node = heapq.heappop(open_list)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            return return_path(current_node)

        # Generate children
        children = []
        
        for new_position in adjacent_squares: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:
            # Child is on the closed list
            if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            if child in open_list: 
                idx = open_list.index(child) 
                if child.g < open_list[idx].g:
                    # update the node in the open list
                    open_list[idx].g = child.g
                    open_list[idx].f = child.f
                    open_list[idx].h = child.h
            else:
                # Add the child to the open list
                heapq.heappush(open_list, child)

    warn("Couldn't get a path to destination")
    return None

def marker(map, size):
  for i in range(0, size):
    for j in range(0, size):
      if map[i][j] > 0 and map[i][j] != 7777: # obstacle maping
        for k in range(-3, 6): # vertical
          for l in range(-4, 5): 
            if (i+k < size and j+l <size) and (i+k > 0 and j+l > 0) and map[i+k][j+l] == 0:
              map[i+k][j+l] = -10
  # for the empty side
  for i in range(0, size):
    map[0][i] = 0
    map[size - 1][i] = 0
    map[i][0] = 0
    map[i][size - 1]= 0
  return map

def align_coordinates(x,y,theta): # to x axis
    radian = theta*math.pi/180
    x_changed = x*math.cos(radian) - y*math.sin(radian)
    y_changed = x*math.sin(radian) + y*math.cos(radian)
    return x_changed, y_changed

def real_to_map(x,y, half, resolution):
  x_return = -(int(x/resolution) - half)
  if x_return > half:
    x_return += 1
  y_return = -(int(y/resolution) - half) 
  if y_return > half:
    y_return += 1
  return x_return, y_return

def main(map, size, half, resolution):
  global map_count
  global next_x_previous, next_y_previous
  start = (half, half) # center = (50,50)
  
  x_goal, y_goal = align_coordinates(Control.x_Goal_now - Control.x_current, Control.y_Goal_now - Control.y_current, -(Control.y))
  x_goal, y_goal = real_to_map(x_goal, y_goal, half, resolution)
  if x_goal < 0:
    x_goal = 0
  if y_goal < 0:
    y_goal = 0
  if x_goal > (size - 1):
    x_goal = (size - 1)
  if y_goal > (size - 1):
    y_goal = (size - 1)
  end = (x_goal, y_goal)

  map = marker(map, size)
  if map[half][half] != 7777:
    path = astar(map, start, end)
  else:
    pass
  if type(path) is not type(None): 
    if len(path) > 1 :
      Control.next_x = path[1][0]
      Control.next_y = path[1][1]
      for i in range(0,len(path)-1):
        x = path[i][0]
        y = path[i][1]
        map[x][y] = -1
      x = path[len(path)-1][0]
      y = path[len(path)-1][1]
      map[x][y] = -100
    # else:
      # print("no path")
  else:
    print("no path")

  return map