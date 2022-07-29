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
from std_msgs.msg import String, Int32MultiArray

from rospy.rostime import Duration, Time


class LapTimer():
    """ 
    """
    def __init__(self, time_step=0.1):
        self.setup_node()
        

    def setup_node(self):
        #Topics & Subs, Pubs
        
        
        lap_sub= '/laps'
        mux_sub='/mux'


        self.lap_sub=rospy.Subscriber(lap_sub, String, self.lap_callback)
        self.mux_sub=rospy.Subscriber(mux_sub, Int32MultiArray, self.mux_callback)

        
    def lap_callback(self, data:String):    
        self.lap_time=Time.now()-self.start_time
        self.collision=0
        rospy.loginfo("Success. Duration: {}s".format(self.lap_time.to_sec()))

    

    def mux_callback(self, data:Int32MultiArray):
        #print(data.data)
        if np.array_equal(data.data, np.array([0,0,0,0,1], dtype=np.int32)):
            self.start_time=Time.now()
            rospy.loginfo("start time:{}".format(self.start_time))
        
        elif np.array_equal(data.data, np.array([0,0,0,0,0], dtype=np.int32)):
            
            self.lap_time=Time.now()-self.start_time
            self.collision=1
        
            rospy.loginfo("Collision. Duration: {}".format(self.lap_time.to_sec()))
        else: 
            raise Exception("weird mux: {}".format(data))
    
    def do_logging(self):
            pass
    

def main(args):
    
    
    rospy.init_node("lap_timer_node", anonymous=True)
    rospy.loginfo("starting up lap_timer node")
    lap_timer=LapTimer()
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)





