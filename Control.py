#!/usr/bin/env python
# -*- coding: utf-8 -*-

from cmath import  pi
import numpy as np
import rospy
import threading
import math
import time

# is it ok?

from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates

import Lidar

A1_velocity = 0.0
location_Mat = np.zeros((3,2))
hight = 0.34
x_current = 0
y_current = 0
linear_x = 0.0
linear_y = 0.0
angular_z = 0.0
r = 0.0
p = 0.0
y = 0.0
direction = ''
direction_previous = ''
direction_count = 0

no_goal = 0
Goals = Goals = [[8.0, 0.0], [0.0, 0.0]]
Goal_num = len(Goals)
Goal_Count = 0
x_Goal_now = Goals[0][0]
y_Goal_now = Goals[0][1]
y_Goal_final = Goals[Goal_num-1][1]
x_Goal_final = Goals[Goal_num-1][0]
Goal_final = Goals[Goal_num-1]

next_x = 0
next_y = 0
ok_to_contact = 0
path_num_previous = 0

y_box = 0

def get_theta(x, y):
    # r = np.sqrt(x**2 + y**2)
    theta = np.arctan2(x, y)*180/pi
    '''if theta < 0:
        theta = 360 + theta'''
    return theta

def align_coordinates(y,x,theta):
    radian = theta*math.pi/180
    y_changed = y*math.cos(radian) - x*math.sin(radian)
    x_changed = y*math.sin(radian) + x*math.cos(radian)
    return y_changed, x_changed

def quaternion_to_euler(quarternion):

    x = quarternion.x
    y = quarternion.y
    z = quarternion.z
    w = quarternion.w

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.degrees(math.atan2(t0, t1))
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.degrees(math.asin(t2))
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.degrees(math.atan2(t3, t4))
    
    return roll_x, pitch_y, yaw_z # in radians

def get_state(states):
    global hight
    global x_current, y_current
    global linear_y, linear_x, angular_z
    global r, p, y
    global y_box
    index_A1 = states.name.index("a1_gazebo")
    hight = states.pose[index_A1].position.z 
    x_current = states.pose[index_A1].position.x
    y_current = states.pose[index_A1].position.y
    quarternion = states.pose[index_A1].orientation
    r, p, y = quaternion_to_euler(quarternion)

    # velocity
    linear_x = states.twist[index_A1].linear.x
    linear_y = states.twist[index_A1].linear.y
    angular_z = states.twist[index_A1].angular.z

    # box
    index_box = states.name.index("dynamic_obstacle")
    y_box = states.pose[index_box].position.y

def which_quadrant(y,x):
    if y*x >= 0:
        if y >= 0:
            return 1
        else:
            return 3
    else:
        if y >= 0:
            return 2
        else:
            return 4

def drection_controller(matrix):
    global x_current, y_current
    global next_x, next_y
    global direction_previous
    returning_direction = ""

    angle = get_theta(-(next_x - half), next_y - half)

    if angle == 90:
        returning_direction = "G"
    elif angle == -90:
        returning_direction = "B"
    elif angle == 0:
        returning_direction = "R_S"
    elif angle == 180:
        returning_direction = "L_S"
    else:
        if abs(angle) < 90:
            returning_direction = "R"
        elif abs(angle) > 90:
            returning_direction = "L"
    
    direction_previous = returning_direction

    # print(direction_count, returning_direction)

    return returning_direction

def is_goal():
    global y_current, x_current
    global y_Goal_now, x_Goal_now
    if abs(y_Goal_now - y_current) < 0.1 and abs(x_current - x_Goal_now) < 0.1:
        return "Y"
    else:
        return "N"

def goal_updater(count):
    global Goals
    global x_Goal_now, y_Goal_now
    y_Goal_now = Goals[count][1]
    x_Goal_now = Goals[count][0]

def A1_go(x,y,z):
    global A1_gazebo_msg

    A1_gazebo_msg.pose.position.z = 0

    A1_gazebo_msg.twist.linear.x = x
    A1_gazebo_msg.twist.linear.y = y
    A1_gazebo_msg.twist.angular.z = z

    A1_gazebo_msg.twist.linear.z = 0

def A1_stay_high():
    global A1_gazebo_msg

    A1_gazebo_msg.pose.position.z = 0.005

def control_loop():
    global hight
    global Goal_Count
    global linear_x, linear_y, angular_z
    global direction, direction_previous 
    global A1_gazebo_msg
    global y_box
    global size, half
    size = Lidar.size
    half = int(size/2)
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rate = rospy.Rate(50) 
    A1_gazebo_msg = ModelState()
    box_msg = ModelState()
    # initialize
    A1_gazebo_msg.model_name = "a1_gazebo"
    A1_gazebo_msg.reference_frame = "a1_gazebo"

    box_msg.model_name = "dynamic_obstacle"
    box_msg.reference_frame = "dynamic_obstacle"
    rospy.Subscriber("/gazebo/model_states", ModelStates, get_state, queue_size=10)
    while not rospy.is_shutdown():
        if hight < 0.35:
            A1_stay_high()
            pub.publish(A1_gazebo_msg)
        else:
            location_Mat[0][0] = location_Mat[1][0]
            location_Mat[0][1] = location_Mat[1][1]
            location_Mat[1][0] = y_current
            location_Mat[1][1] = x_current
            location_Mat[2][0] = y_Goal_now
            location_Mat[2][1] = x_Goal_now
          
            direction_previous = direction
            direction = drection_controller(location_Mat)

            # print(direction)

            if direction == "L":
                A1_go(0, 0, A1_velocity*2)
                pub.publish(A1_gazebo_msg)
            elif direction == "R":
                A1_go(0, 0, -A1_velocity*2)
                pub.publish(A1_gazebo_msg)
            elif direction == "G":
                A1_go(A1_velocity,0,0)
                pub.publish(A1_gazebo_msg)
            elif direction == "B":
                A1_go(-A1_velocity,0,0)
                pub.publish(A1_gazebo_msg)
            elif direction == "R_S":
                A1_go(0, -A1_velocity, 0)
                pub.publish(A1_gazebo_msg)
            elif direction == "L_S":
                A1_go(0, A1_velocity ,0)
                pub.publish(A1_gazebo_msg)
            else:
                pass

            if is_goal() == "Y":
                if Goal_Count < Goal_num:
                    Goal_Count += 1
                if Goal_Count == Goal_num:
                    A1_go(0,0,0)
                    pub.publish(A1_gazebo_msg)
                else:
                    goal_updater(Goal_Count)
        
        box_msg.twist.linear.y = 0.9
        pub.publish(box_msg)

      
        rate.sleep()
