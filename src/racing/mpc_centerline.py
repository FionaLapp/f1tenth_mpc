#!/usr/bin/env python3
"""
Created on Thu May 12 12:06:18 2022
@author: Fiona
"""
from __future__ import print_function
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import interpolate

#MPC imports
# Add do_mpc to path. This is not necessary if it was installed via pip.
import sys
from helper.visualiser import GapMarker
sys.path.append('../../')




#ROS Imports
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import mpc_base_code as mpc_base_code

class ReadCSVController(mpc_base_code.BaseController):
    """ 
    """
    def __init__(self, time_step=0.1):
        super().__init__( time_step=time_step)
        
        
        
       

    

    def localisation_callback(self, data:Odometry):
        """
        Could be from any source of localisation (e.g. odometry or lidar)---> adapt get_state_from_data metod accordingly
        """
        try:#update current state
            
            self.previous_delta=self.state[2]
            x=data.pose.pose.position.x
            y=data.pose.pose.position.y
            orientation_list=[data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
            (roll, pitch, phi) = euler_from_quaternion (orientation_list)
            #rospy.loginfo("{}, {}, {}".format(x, y, phi))
            self.state= np.array([x,y, phi])
            
            #update target: use the distance already travelled and look it's index up in the csv data, then, in the tvp template, use the index to find the next target points
            distances_to_current_point=(self.path_data_x-self.state[0])**2+(self.path_data_y-self.state[1])**2
            closest=(distances_to_current_point.argmin()+2) #not actually the closest because we want to always be ahead
            self.index= closest %self.path_length
            if closest ==self.path_length:
                self.laps_completed+=1
                rospy.loginfo("Yay, you made it! {} laps!".format(self.laps_completed))
            #rospy.loginfo(self.index)
            self.make_mpc_step(self.state)
        except AttributeError:
            pass
            #rospy.loginfo("Initialisation not finished")

    
    @property
    def stage_cost(self):
        """
        none
        """
        return (self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2 +2.3*self.curvature*self.v #+(200/self.wall_distance)*self.v
         
  

def main(args):
    
    
    rospy.init_node("mpc_node", anonymous=True)
    rospy.loginfo("starting up mpc node")
    time_step=0.1
    model_predictive_control =ReadCSVController(time_step=time_step)
    rospy.sleep(time_step)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)





