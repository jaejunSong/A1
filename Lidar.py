#!/usr/bin/env python
# -*- coding: utf-8 -*-

from concurrent.futures import thread
import numpy as np
import sys
import os
import time
import threading
import ros_numpy

import sensor_msgs.point_cloud2 as pc2

import Path
import Control
import UI

np.set_printoptions(threshold=sys.maxsize)

height = 0.20
size = 81
resolution = 0.05 # m
half = int(size/2)
threshold = 0
frame_count = 0
is_go = 0

return_map_previous = np.zeros(31)

L_moving_obstacle = 0
R_moving_obstacle = 0

map_final = np.zeros([size,size]) 

def set_threshold(map):
    for i in range(0,size):
        for j in range(0,size):
            if map[i][j] < threshold:
                map[i][j] = 0
    return map

def real_to_map(x,y):
  x_return = -(int(x/resolution) - half)
  if x_return > half:
    x_return += 1
  y_return = -(int(y/resolution) - half) 
  if y_return > half:
    y_return += 1
  return x_return, y_return

def block_right(map):
    for i in range(0, int(half/4)):
        map[half - 2 + i][half + i + 2] = 7777
        map[half - 2 + i][half + i + 1] = 7777
    return map

def block_left(map):
    for i in range(0, int(half/4)):
        map[half - 2 + i][half - i - 2] = 7777
        map[half - 2 + i][half - i - 1] = 7777
    return map

def write_map(data):
    global is_go, frame_count
    global return_map_previous, map_final
    global L_moving_obstacle, R_moving_obstacle
    map = np.zeros([size,size])
    # xyz_array = ros_numpy.point_cloud2.converts_to_numpy(data)
    # print(xyz_array)
    pcd_data = list(pc2.read_points(data, skip_nans=True, field_names = ("x", "y", "z")))
    for point in pcd_data:
        z = point[2]
        if abs(z) < height and abs(point[0]) < 2.0 and abs(point[1]) < 2.0:
            x = int(point[0]/resolution)
            y = int(point[1]/resolution)
            map[-x + half][-(y + half)] += 1
    map = set_threshold(map)
    if (Control.direction == 'L' or Control.direction == "L_S"):
        map = block_right(map)
    if (Control.direction == 'R' or Control.direction == "R_S"):
        map = block_left(map)
    # map = block_back(map)

    map = Path.main(map, size, half, resolution)
    # os.system('clear')
    map_str = ''
    for i in range(0,size):
        for j in range(0,size):
            if i == half and j == half:
                map_str += '▦ '
            elif map[i][j] > 0 and map[i][j] != 7777:
                map_str += '■ ' # obstaclesss
            elif map[i][j] == 0: 
                map_str += '□ ' # no obstacle
            elif map[i][j] == -10: 
                # map_str += '□ ' # no obstacle
                map_str += '▣ ' # can't go
            elif map[i][j] == -1:
                map_str += '▦ ' # path
                # map_str += '□ '
            elif map[i][j] == -100:
                # map_str += '□ '
                map_str += '◙ ' # goal
            elif map[i][j] == 7777:
                map_str += '□ '
                # map_str += '▦ ' # direction
        # print(map_str)
        map_str += '\n'

    UI.map.setText(str(map_str))
