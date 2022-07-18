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
    def __init__(self, max_speed=None, use_splines=False, time_step=0.1):
        self.read_desired_path(use_splines=use_splines)
        super().__init__(max_speed=max_speed, time_step=time_step)
        
        
        
       

    def read_desired_path(self, use_splines):
        pathfile_name=rospy.get_param('/mpc/directory')+'/src/maps/Sochi/Sochi_centerline.csv'
        self.path_data=pd.read_csv(pathfile_name)
        self.path_length=self.path_data.shape[0]
        self.path_data_x=self.path_data[' x_m'].to_numpy()
        self.path_data_y=self.path_data[' y_m'].to_numpy()   
        delta_x=self.path_data_x-np.roll(self.path_data_x, 1) 
        delta_y=self.path_data_y-np.roll(self.path_data_y, 1)   
        delta_s=np.sqrt(delta_x**2+delta_y**2)     
        self.path_data_s= np.cumsum(delta_s)                                    
        self.previous_x=0
        self.previous_y=0
        self.previous_delta=0
        self.distance_travelled=0.0
        self.index=0

        #the following code was just me trying to compute splines instead of only having  points and connecting them with lines. probably unecessary
        self.use_splines=use_splines
        if use_splines:
            spline=interpolate.SmoothBivariateSpline(self.path_data_x, self.path_data_y, self.path_data_s) 
            n_1 = len(self.path_data_y)
            tck,u = interpolate.splprep([self.path_data_x,self.path_data_y],s=0)
            unew = np.arange(0,1.0,0.001)
            out = interpolate.splev(unew,tck)
            
            plt.figure()
            plt.plot(out[0],out[1])#,np.sin(2*np.pi*unew),np.cos(2*np.pi*unew),'b')
            plt.legend(['Cubic Spline'])
            
            plt.title('Spline of parametrically-defined curve')
            plt.show()
            g=GapMarker(out[0],out[1])
            self.out=out
            g.draw_point()
    

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
            
            self.index=distances_to_current_point.argmin()+ 11 %len(self.path_data_x)
            #rospy.loginfo(self.index)
            self.make_mpc_step(self.state)
        except AttributeError:
            print("Initialisation not finished")

    
    @property
    def stage_cost(self):
        """
        none
        """
        return (self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2 #+13*self.measured_steering_angle*self.v #+(200/self.wall_distance)*self.v
         
  

def main(args):
    
    
    rospy.init_node("mpc_node", anonymous=True)
    rospy.loginfo("starting up mpc node")
    time_step=0.1
    model_predictive_control =ReadCSVController(max_speed=None, use_splines=False, time_step=time_step)
    rospy.sleep(time_step)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)





