#!/usr/bin/env python3
"""
Created on Thu May 12 12:06:18 2022
@author: Fiona
"""
from __future__ import print_function
import sys
import math
import numpy as np
import pandas as pd
import os

#ROS Imports
import rospy
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker



class InitialiseRobot:
    def __init__(self):

        
        
        path_topic='/goal_path'
        initial_pose_topic='/initialpose'
        self.path_sub=rospy.Subscriber(path_topic, Path, self.path_callback)#TODO: Subscribe to LIDAR
        self.initial_pose_pub = rospy.Publisher(initial_pose_topic, PoseStamped, queue_size=1)#TODO: Publish to drive
        
    def path_callback(self, data):

        pose=data[0]
        self.initial_pose_pub.publish(pose)
    



def main(args):
    
    
    rospy.init_node("initial_pose_node")
    rospy.loginfo("starting up initial pose node")
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)






'''

for k in range(10):
  u0 = mpc.make_step(x0)
  x0 = simulator.make_step(u0)

fig, ax, graphics = do_mpc.graphics.default_plot(mpc.data, figsize=(16,9))
graphics.plot_results()
graphics.reset_axes()
plt.show()


fig, ax = plt.subplots(3, figsize=(16,9))


sim_graphics.add_line(var_type='_x', var_name='x', axis=ax[0])
sim_graphics.add_line(var_type='_x', var_name='y', axis=ax[0])


mpc_graphics.add_line(var_type='_u', var_name='a', axis=ax[1])


mpc_graphics.add_line(var_type='_u', var_name='delta', axis=ax[2])
'''

