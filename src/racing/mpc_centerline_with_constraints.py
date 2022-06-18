#!/usr/bin/env python3
"""
Created on Thu May 12 12:06:18 2022
@author: Fiona
"""
from __future__ import print_function
import sys
import numpy as np
import pandas as pd


#MPC imports
# Add do_mpc to path. This is not necessary if it was installed via pip.
import sys
sys.path.append('../../')



#ROS Imports
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import mpc_base_code as mpc_base_code
import helper.visualiser as visualiser
class ReadCSVController(mpc_base_code.BaseController):
    """ 
    """
    def __init__(self):
        self.read_desired_path()
        super().__init__()
        
        
        
       


    def read_desired_path(self):
        pathfile_name=rospy.get_param('/mpc/directory')+'/src/maps/Sochi/Sochi_centerline.csv'
        self.path_data=pd.read_csv(pathfile_name)
        #print(self.path_data.keys())
        self.path_length=self.path_data.shape[0]
        self.path_data_x=self.path_data[' x_m'].to_numpy()
        self.path_data_y=self.path_data[' y_m'].to_numpy()                                            
        self.previous_x=0
        self.previous_y=0
        self.distance_travelled=0.0
        self.index=0
        self.trackwidth=1.1
        self.calculate_wall_points()

    def  calculate_wall_points(self):
        """ take the corresponding centerline point, create a vector perpendicular to it by switching 
        coordinates and either sign (so the dot prodict is 0), divide by length to get unit vector, multiply 
        by half the track width. add that vector to the original centerline point.
        """
        
        
        self.path_tangent_x= self.path_data_x-np.roll(self.path_data_x, 1)
        self.path_tangent_y= self.path_data_y-np.roll(self.path_data_y, 1)
        l=np.sqrt(self.path_tangent_x**2+self.path_tangent_y**2)
        
        self.path_data_x_r=self.path_data_x+self.path_tangent_y*self.trackwidth/l
        self.path_data_y_r=self.path_data_y-self.path_tangent_x*self.trackwidth/l
        self.path_data_x_l=self.path_data_x-self.path_tangent_y*self.trackwidth/l
        self.path_data_y_l=self.path_data_y+self.path_tangent_x*self.trackwidth/l
        


    def localisation_callback(self, data:Odometry):
        """
        Could be from any source of localisation (e.g. odometry or lidar)---> adapt get_state_from_data metod accordingly
        """
        try:#update current state
            self.previous_x=self.state[0]
            self.previous_y=self.state[1]
            x=data.pose.pose.position.x
            y=data.pose.pose.position.y
            orientation_list=[data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
            (roll, pitch, phi) = euler_from_quaternion (orientation_list)
            #rospy.loginfo("{}, {}, {}".format(x, y, phi))
            self.state= np.array([x,y, phi])
            
            #update target: use the distance already travelled and look it's index up in the csv data, then, in the tvp template, use the index to find the next target points
            distances_to_current_point=(self.path_data_x-self.state[0])**2+(self.path_data_y-self.state[1])**2
            
            self.index=distances_to_current_point.argmin()+2 %1170
            #rospy.loginfo(self.index)
            self.make_mpc_step(self.state)
            m=visualiser.GapMarker(self.path_data_x_l, self.path_data_y_l, 10)
            m.draw_point()
        except AttributeError:
            print("Initialisation not finished")

    
        
  
def main(args):
    
    
    rospy.init_node("mpc_node", anonymous=True)
    rospy.loginfo("starting up mpc node")
    model_predictive_control =ReadCSVController()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)





