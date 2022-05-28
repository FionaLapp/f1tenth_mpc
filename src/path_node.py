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



class DesiredPath:
    def __init__(self):

        
        pathfile_name=rospy.get_param('/path_node/directory')+'/src/maps/Sochi/Sochi_raceline.csv'
        path_data=pd.read_csv(pathfile_name)
        #rospy.loginfo(path_data.head())
        #Topics & Subs, Pubs
        path_topic='/goal_path'
        rospy.loginfo(path_data.keys())
        self.path_pub=rospy.Publisher(path_topic, Path, queue_size=1)
        self.path=Path()
        self.path.header.frame_id='map'
        for i in range(path_data.shape[0]):
            pose=PoseStamped()
            pose.pose.position.x=path_data[' x_m'][i]
            pose.pose.position.y=path_data[' y_m'][i]
            self.path.poses.append(pose)
    



def main(args):
    
    
    rospy.init_node("path_node")
    rospy.loginfo("starting up path node")

    desired_path = DesiredPath()
    rate=rospy.Rate(2)
    counter=0
    path_length=len(desired_path.path.poses)
    while not rospy.is_shutdown():
        current_path=Path()
        current_path.header.frame_id='map'
        current_path.poses=desired_path.path.poses[(counter)%path_length:(counter+20)%path_length]
        
        desired_path.path_pub.publish(current_path)
        #desired_path.path_pub.publish(desired_path.path)
        rate.sleep()
        counter+=1

if __name__=='__main__':
	main(sys.argv)






