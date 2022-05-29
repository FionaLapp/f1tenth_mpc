#!/usr/bin/env python3
"""
Created on Thu May 12 12:06:18 2022
@author: Fiona
"""
from __future__ import print_function
from re import X
import sys
import math
import numpy as np
import pandas as pd
import os

#ROS Imports
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker



class Echo:
    def __init__(self):

        
        echo_topic='/odom'
        echo_sub=rospy.Subscriber(echo_topic, Odometry, self.echo_callback)

    def echo_callback(self, data):
        rospy.loginfo(data)
    



def main(args):
    
    
    rospy.init_node("echonode")
    rospy.loginfo("starting up echo node")
    echo=Echo()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)






