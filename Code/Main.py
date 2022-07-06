#!/usr/bin/env python
# -*- coding: utf-8 -*-

#####################################################
## Jaejun ## CAMEL ## PNU EE ## 
## First released ## 2022/05/26 ##
## Last modified ## 2022/07/06 ## 
#####################################################

import rospy
import threading

from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import PointCloud2

import Lidar
import Control
import UI
import Path

def value_setting():
    # Set A1's Speed
    Control.A1_velocity = 0.0
    # Set A1's Goal [x,y] now max is 5.0m

def subscribe_Thread():

    rospy.Subscriber("/velodyne_points", PointCloud2, Lidar.write_map, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        
        rospy.init_node('controller',anonymous=True)

        value_setting()

        subscribe_thread = threading.Thread(target=subscribe_Thread)
        subscribe_thread.daemon = True
        subscribe_thread.start()

        visual_thread = threading.Thread(target=UI.UI_Thread)
        visual_thread.daemon = True
        visual_thread.start()

        # main
        Control.control_loop()

    except rospy.ROSInterruptException:
        pass