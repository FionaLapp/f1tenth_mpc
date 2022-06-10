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
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import mpc_base_code as mpc_base_code

class FTGController(mpc_base_code.BaseController):
    """ 
    """
    def __init__(self):
        super().__init__()
        
    def  setup_node(self) :
        super().setup_node()

        laser_topic= '/scan'
        laser_sub=rospy.Subscriber(laser_topic, LaserScan, self.scan_callback)
        
    def  scan_callback(self, data):
        pass


    def read_desired_path(self):
        pathfile_name=rospy.get_param('/mpc/directory')+'/src/maps/Sochi/Sochi_centerline.csv'
        self.path_data=pd.read_csv(pathfile_name)
        print(self.path_data.keys())
        self.path_length=self.path_data.shape[0]
        self.path_data_x=self.path_data[' x_m'].to_numpy()
        self.path_data_y=self.path_data[' y_m'].to_numpy()                                            
        self.previous_x=0
        self.previous_y=0
        self.distance_travelled=0.0
        self.index=0
    

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
            
            self.index=distances_to_current_point.argmin()+5 %1170
            #rospy.loginfo(self.index)
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
        proc_ranges = np.where[ranges_convolved<=3, ranges_convolved, 3] #set everything larger than 3 to 3
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
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        gap_array=ranges[start_i: end_i]
        furthest_point_index=np.argmax[gap_array]
        return None #TODOsss

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)

        #Find closest point to LiDAR
        closest_point_index=np.argmin[proc_ranges]

        #Eliminate all points inside 'bubble' (set them to zero) 
        bubble_radius=3
        proc_ranges[closest_point_index-bubble_radius:closest_point_index+bubble_radius]=0
 
        #Find max length gap 
        start_index, end_index=self.find_max_gap(proc_ranges)

        #Find the best point in the gap 
        point=self.find_best_point(start_index, end_index, proc_ranges)
        #Publish Drive message
    
        
  

def main(args):
    
    
    rospy.init_node("mpc_node", anonymous=True)
    rospy.loginfo("starting up mpc node")
    model_predictive_control =FTGController()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)





