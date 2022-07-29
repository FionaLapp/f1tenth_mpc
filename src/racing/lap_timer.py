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
from std_msgs.msg import String, Int32MultiArray, Int32

from rospy.rostime import Duration, Time
import os


class LapTimer():
    """ 
    """
    def __init__(self, time_step=0.1):
        self.lap_count=0
        self.collision=0
        namespace=rospy.get_namespace()+'lap_timer/'
        self.max_laps=rospy.get_param(namespace+'max_laps')
        self.setup_node()
        

    def setup_node(self):
        #Topics & Subs, Pubs
        
        
        lap_sub= '/laps'
        mux_sub='/mux'


        self.lap_sub=rospy.Subscriber(lap_sub, Int32, self.lap_callback)
        self.mux_sub=rospy.Subscriber(mux_sub, Int32MultiArray, self.mux_callback)

        
    def lap_callback(self, data:Int32):    
        if self.lap_count<self.max_laps-1:
            self.lap_time=Time.now()-self.start_time
            self.lap_count+=1
            rospy.loginfo("Success: {} lap(s) completed. Duration: {}s".format(self.lap_count, self.lap_time.to_sec()))
        elif self.lap_count==self.max_laps-1:
            self.lap_time=Time.now()-self.start_time
            self.lap_count+=1
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
            
            self.lap_time=Time.now()-self.start_time
            self.collision+=1
        
            rospy.loginfo("Collision. {} lap(s) completed before crash. Duration: {}".format(self.lap_count, self.lap_time.to_sec()))
            self.do_logging()
        else: 
            raise Exception("weird mux: {}".format(data))
    
    def do_logging(self): #write to file, then exit
        #TODO write to file
        #os.system("rosnode kill -a")
        os.system("killall -9 rosmaster")
    

def main(args):
    
    
    rospy.init_node("lap_timer_node", anonymous=True)
    rospy.loginfo("starting up lap_timer node")
    lap_timer=LapTimer()
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)





