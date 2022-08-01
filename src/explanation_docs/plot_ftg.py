#!/usr/bin/env python3
"""
Created on Thu May 12 12:06:18 2022
@author: Fiona


To get ranges for plotting (for report, not necessary for running controllers)
add  <node pkg="f1tenth_mpc" name="plotter" type="plot_ftg.py" output="screen"></node>
to launch file
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys
import csv
#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class Plotter():
    """ 
    """
    def __init__(self, add_markers=True, time_step=0.1):
        self.setup_nodes()
        
        
        

    def  setup_nodes(self) :
        laser_topic= '/scan'
        laser_sub=rospy.Subscriber(laser_topic, LaserScan, self.lidar_callback)
        
        key_topic= '/key'
        key_sub=rospy.Subscriber(key_topic, String, self.key_callback)
        
    def key_callback(self, data:String):
        if data.data=="p":
            self.make_plots()
    def make_plots(self):
        self.preprocess_lidar(self.lidar_data)
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        cwd=sys.path[0]
        print(cwd)
        csv_filepath=cwd
        with open(cwd+"/ftg_ranges_example.csv", "w")as f:
            writer = csv.writer(f)

                 # write a row to the csv file
            writer.writerow(ranges)
        #print(ranges, file="ftg_ranges_example")
        print("done")

   
    
    def lidar_callback(self, data:LaserScan):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        self.amin=data.angle_min
        self.amax=data.angle_max
        self.angle_increment=data.angle_increment
        self.lidar_data=data.ranges
    
def main(args):
    
    
    rospy.init_node("mpc_node", anonymous=True)
    rospy.loginfo("starting up mpc node")
    
    plotter=Plotter()
    #rospy.Timer(rospy.Duration(30), model_predictive_control.plot_mpc)
    
    #rospy.sleep(time_step)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)

