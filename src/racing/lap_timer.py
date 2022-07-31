#!/usr/bin/env python3
"""
Created on Thu May 12 12:06:18 2022
@author: Fiona
"""
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt




#ROS Imports
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String, Int32MultiArray, Int32

from rospy.rostime import Duration, Time
import os
import time
import csv


class LapTimer():
    """ 
    """
    def __init__(self, time_step=0.1):
        self.lap_count=0
        self.collision=0
        namespace=rospy.get_namespace()+'lap_timer/'
        self.max_laps=rospy.get_param(namespace+'max_laps')
        self.x_start=rospy.get_param(namespace+'x_start')
        self.y_start=rospy.get_param(namespace+'y_start')
        self.w_start=rospy.get_param(namespace+'w_start')
        self.z_start=rospy.get_param(namespace+'z_start')
        self.penalty=rospy.get_param(namespace+'penalty')
       
        self.log_file_name=rospy.get_param(namespace+'log_file_name', default=None)
        if self.log_file_name is not None:
            self.node_type=rospy.get_param(namespace+'node_type')
            self.r_v=rospy.get_param(namespace+'r_v')
            self.r_delta=rospy.get_param(namespace+'r_delta')
            self.n_horizon=rospy.get_param(namespace+'n_horizon')
            self.velocity=rospy.get_param(namespace+'velocity')
            self.world_name=rospy.get_param(namespace+'world_name')
            self.include_obstacles=rospy.get_param(namespace+'include_obstacles')
        self.laps=[]
        self.setup_node()
        

    def setup_node(self):
        #Topics & Subs, Pubs
        
        
        lap_sub= '/laps'
        mux_sub='/mux'
        initialpose_pub='/initialpose'
        key_pub='/key'


        self.lap_sub=rospy.Subscriber(lap_sub, Int32, self.lap_callback)
        self.mux_sub=rospy.Subscriber(mux_sub, Int32MultiArray, self.mux_callback)
        self.initialpose_pub=rospy.Publisher(initialpose_pub, PoseWithCovarianceStamped, queue_size=5)
        self.key_pub=rospy.Publisher(key_pub, String, queue_size=5)

        
    def lap_callback(self, data:Int32):    
        if self.lap_count<self.max_laps-1:
            self.lap_time=Time.now()-self.start_time
            self.lap_count+=1
            self.laps.append(self.lap_time.to_sec())
            rospy.loginfo("Success: {} lap(s) completed. Duration: {}s".format(self.lap_count, self.lap_time.to_sec()))
        elif self.lap_count==self.max_laps-1:
            self.lap_time=Time.now()-self.start_time
            self.lap_count+=1
            self.laps.append(self.lap_time.to_sec())
            rospy.loginfo("Last lap complete. Success: {} lap(s) completed. Duration: {}s".format(self.lap_count, self.lap_time.to_sec()))
            self.do_logging()
        else: #this shouldn't happen
            rospy.logwarn("Counted up to {} laps even though I should stop at {}. Something might've gone wrong.".format(self.lap_count+1, self.max_laps))
            self.do_logging()
    

    def mux_callback(self, data:Int32MultiArray):
        #print(data.data)
        if np.array_equal(data.data, np.array([0,0,0,0,1], dtype=np.int32)):
            self.start_time=Time.now()
            rospy.loginfo("start time:{}".format(self.start_time))
        
        elif np.array_equal(data.data, np.array([0,0,0,0,0], dtype=np.int32)):
            lap_time=self.start_time-Time.now() #this is just fyi, the timer keeps running
            rospy.loginfo("Collision. Resetting to initial position and adding {}s penalty. Wasted time on lap: {}".format(self.penalty, lap_time.to_sec()))
            
            self.lap_time=Time.now()-self.start_time
            self.collision+=1
            initialpose=PoseWithCovarianceStamped()
            initialpose.pose.pose.position.x=self.x_start
            initialpose.pose.pose.position.y=self.y_start
            initialpose.pose.pose.orientation.w=self.w_start
            initialpose.pose.pose.orientation.z=self.z_start
            self.initialpose_pub.publish(initialpose)
            time.sleep(self.penalty)
            self.key_pub.publish("n")
            rospy.loginfo("Collision. {} lap(s) completed before crash. Duration: {}".format(self.lap_count, self.lap_time.to_sec()))
            #self.do_logging()
        else: 
            raise Exception("weird mux: {}".format(data))
    
    def do_logging(self): #write to file, then exit
        
        if not self.log_file_name is None: #if the logfile doesn't exist (because the thing isn't called from the python script), we just don't log. Brains, eh?
            #TODO write to file
            #note: header is ["node_type","r_v", "r_delta", "n_horizon", "velocity", "world_name", "include obstacles", "lap1", "lap2","lap3", "lap4", "lap5", "collisions", "dnf", "mean_laptime", "std_dev"]

            line_params=[self.node_type, self.r_v, self.r_delta, self.n_horizon, self.velocity, self.world_name, self.include_obstacles]
           
            dnf=(self.lap_count<self.max_laps)
            if not len(self.laps)==0:
                mean_laptime=np.mean(np.array(self.laps))
                std_dev=np.std(np.array(self.laps))
            else:
                mean_laptime="-"
                std_dev="-"
            
            calculations=[self.collision, dnf, mean_laptime, std_dev]
            
            for i in range (self.max_laps-self.lap_count):
                self.laps.append("-")
            times=self.laps
            
            line=line_params+ times+calculations
            # write a row to the csv file
            
            with open(self.log_file_name, 'a') as f:
                # create the csv writer
                writer = csv.writer(f)

                 # write a row to the csv file
                writer.writerow(line)
        os.system("killall -9 rosmaster")
    

def main(args):
    
    
    rospy.init_node("lap_timer_node", anonymous=True)
    rospy.loginfo("starting up lap_timer node")
    lap_timer=LapTimer()
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)





