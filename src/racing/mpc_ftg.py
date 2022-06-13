#!/usr/bin/env python3
"""
Created on Thu May 12 12:06:18 2022
@author: Fiona
"""
from __future__ import print_function
from cmath import inf
import sys
import numpy as np
import pandas as pd


#MPC imports
# Add do_mpc to path. This is not necessary if it was installed via pip.
import sys
sys.path.append('../../')



#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import mpc_base_code as mpc_base_code
import helper.visualiser as visualiser

class FTGController(mpc_base_code.BaseController):
    """ 
    """
    def __init__(self):
        self.t_x=0
        self.t_y=0
        self.state=[0,0,0]
        self.laser_data=None
        super().__init__()
        self.setup_laser_scan()
        
        

    def  setup_laser_scan(self) :
        laser_topic= '/scan'
        laser_sub=rospy.Subscriber(laser_topic, LaserScan, self.lidar_callback)
        #super().setup_node()

        
       

    

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
            self.make_mpc_step(self.state)
        except AttributeError:
            print("Initialisation not finished")
    
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        kernel_size = 4
        kernel = np.ones(kernel_size) / kernel_size
        ranges_convolved = np.convolve(ranges, kernel, mode='same') #averaging over every 4 elements
        max_range=7
        proc_ranges = np.where(ranges_convolved<=max_range, ranges_convolved, max_range) #set everything larger than 3 to 3
        return proc_ranges

    def find_max_gap(self, ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        gap_index_array=np.argwhere(ranges<3) #indices of all elements that aren't gaps
        gap_index_array=gap_index_array.flatten()
        gap_size=gap_index_array[1:]-gap_index_array[:-1] #find the difference between consecutive gap-indices (if thiis is 1, they are adjacent, if it is 2, there is a gap of size 1, etc.)
        
        index_in_gap_index_array=np.argmax(gap_size)#find the index of the largest gap (this index refers to the gap_index_array)
        start_index=gap_index_array[index_in_gap_index_array]+1 #the gap starts one index after the non-gap
        end_index=gap_index_array[index_in_gap_index_array+1] # the gap ends at the next non-gap
        return start_index, end_index
    
    def find_best_point_index(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        gap_array=range
        return int((end_i-start_i)/2+start_i)
        # furthest_point_index=np.argmax(gap_array)+start_i
        # return  furthest_point_index

    def lidar_to_xy(self, index):
        if self.laser_data is None:
            print("no laser data provided")
        else:
            laser_angle = (index * self.laser_data.angle_increment) + self.laser_data.angle_min
            heading_angle=self.state[2]
            rotated_angle = laser_angle + heading_angle
            p_x = self.proc_ranges[index] * np.cos(rotated_angle) + self.state[0]
            p_y = self.proc_ranges[index]  * np.sin(rotated_angle) + self.state[1]
            return p_x, p_y
        


    def lidar_callback(self, data:LaserScan):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        self.laser_data=data
        ranges = data.ranges

        self.proc_ranges = self.preprocess_lidar(ranges)

        #Find closest point to LiDAR
        closest_point_index=np.argmin(self.proc_ranges)

        #Eliminate all points inside 'bubble' (set them to zero) 
        bubble_radius=3
        self.proc_ranges[closest_point_index-bubble_radius:closest_point_index+bubble_radius]=0
 
        #Find max length gap 
        start_index, end_index=self.find_max_gap(self.proc_ranges)

        #Find the best point in the gap 
        best_point_index=self.find_best_point_index(start_index, end_index, self.proc_ranges)
        
        #calculate x and y value, given the index of the lidar point

        
        self.t_x, self.t_y=self.lidar_to_xy(best_point_index)
        gap_x_array=[]
        gap_y_array=[]
        for i in range(start_index, end_index):
            temp_x, temp_y=self.lidar_to_xy(i)
            gap_x_array.append(temp_x)
            gap_y_array.append(temp_y)
        gap_line=visualiser.GapMarker(gap_x_array, gap_y_array, 1)
        # start_x, start_y=self.lidar_to_xy(self.proc_ranges, start_index)
        # end_x, end_y=self.lidar_to_xy(self.proc_ranges, end_index)
        # gap_line=visualiser.GapMarker([start_x, end_x], [start_y, end_y], 1)
        #print("x:{}, y:{}".format(self.t_x, self.t_y))
        #vis_point=visualiser.TargetMarker(self.path_data_x[self.index:self.index+self.n_horizon], self.path_data_y[self.index:self.index+self.n_horizon], 1)
        gap_line.draw_point()
        gap_point=visualiser.TargetMarker(self.t_x, self.t_y, 1)
        #print("x:{}, y:{}".format(self.t_x, self.t_y))
        #vis_point=visualiser.TargetMarker(self.path_data_x[self.index:self.index+self.n_horizon], self.path_data_y[self.index:self.index+self.n_horizon], 1)
        gap_point.draw_point()

    def prepare_goal_template(self, t_now):
        
        template = self.controller.get_tvp_template()

        for k in range(self.n_horizon + 1):
            
            template["_tvp", k, "target_x"]=self.t_x
            template["_tvp", k, "target_y"] =self.t_y
        
        return template     
  

def main(args):
    
    
    rospy.init_node("mpc_node", anonymous=True)
    rospy.loginfo("starting up mpc node")
    model_predictive_control =FTGController()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)





