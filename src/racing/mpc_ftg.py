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
    def __init__(self, add_markers=True, max_speed=None, max_range=2, bubble_radius=10, threshold=2):
        self.params=super().get_params()
        self.setup_finished=False
        self.add_markers=add_markers
        self.setup_laser_scan()
        self.t_x=0
        self.t_y=0
        self.state=[0,0,0]
        if max_range<threshold:
            raise Exception("max range needs to be greater or equal threshold")
        self.max_range=max_range
        self.bubble_radius=bubble_radius
        self.threshold=threshold
        self.cutoff_per_side=150 #so the target point isn't right behind the car
        self.angle_increment=0.005823155865073204 #can I get this from some param file? probably. would that be cleaner? certainly. Will I bother? Doesn't look likely
        self.raw_scan_angle_min=-3.1415927410125732 #could just say -np.pi, but where'd be the fun in that?
        self.angle_min=self.raw_scan_angle_min+(self.angle_increment*self.cutoff_per_side)
        self.min_gap_width= self.params['width']/(self.angle_increment*self.max_range) #min_gap_width=car_width=angle_inc*min_gap_number*max_range (approximately, for large max_ranges  since that'd give the arclength)
        super().setup_node()
        if max_speed is None:
            max_speed=self.params['max_speed']
        super().setup_mpc(max_speed=max_speed, n_horizon=self.params['n_horizon'])
        self.setup_finished=True
        
        

    def  setup_laser_scan(self) :
        laser_topic= '/scan'
        laser_sub=rospy.Subscriber(laser_topic, LaserScan, self.lidar_callback)
        
    def localisation_callback(self, data:Odometry):
        """
        Could be from any source of localisation (e.g. odometry or lidar)---> adapt get_state_from_data metod accordingly
        """
        if self.setup_finished:#update current state
            self.previous_x=self.state[0]
            self.previous_y=self.state[1]
            x=data.pose.pose.position.x
            y=data.pose.pose.position.y
            orientation_list=[data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
            (roll, pitch, phi) = euler_from_quaternion (orientation_list)
            #rospy.loginfo("{}, {}, {}".format(x, y, phi))
            self.state= np.array([x,y, phi])
            self.make_mpc_step(self.state)
        else:
            rospy.logdebug("Initialisation not finished")
    
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        ranges=ranges[self.cutoff_per_side:len(ranges)-self.cutoff_per_side]
        kernel_size = 4
        kernel = np.ones(kernel_size) / kernel_size
        ranges_convolved = np.convolve(ranges, kernel, mode='same') #averaging over every 4 elements
        proc_ranges = np.where(ranges_convolved<=self.max_range, ranges_convolved, self.max_range) #set everything larger than max_range to max_range
        return proc_ranges

    def find_max_gap(self, ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        gap_index_array=np.argwhere(ranges<self.threshold) #indices of all elements that aren't gaps
        gap_index_array=gap_index_array.flatten()
        gap_size=gap_index_array[1:]-gap_index_array[:-1] #find the difference between consecutive gap-indices (if thiis is 1, they are adjacent, if it is 2, there is a gap of size 1, etc.)
        if len(gap_size)==0:
            rospy.loginfo("no walls found near car, will continue going straight")
            return 0, len(ranges)
        index_in_gap_index_array=np.argmax(gap_size)#find the index of the largest gap (this index refers to the gap_index_array)
        start_index=gap_index_array[index_in_gap_index_array]+1 #the gap starts one index after the non-gap
        end_index=gap_index_array[index_in_gap_index_array+1] # the gap ends at the next non-gap
        return start_index, end_index
    
    def find_best_point_index(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        #gap_array=ranges
        if len(ranges)< self.min_gap_width: # this is the minimum of range points needed if the obstacle is at max_range distance. For closer obstacles, more points will be needed
            rospy.loginfo("largest gap is not wide enough")
        return int((end_i-start_i)/2+start_i)
        #furthest_point_index=np.argmax(gap_array)+start_i
        #return  furthest_point_index

    def lidar_to_xy(self, index):
        if self.lidar_data is None:
            raise Warning("no laser data provided")
        else:
            laser_angle = (index * self.angle_increment) + self.angle_min 
            heading_angle=self.state[2]
            point_angle = laser_angle + heading_angle
            p_x = self.proc_ranges[index] * np.cos(point_angle) + self.state[0] +0.265*np.cos(heading_angle)
            p_y =self.proc_ranges[index]  * np.sin(point_angle) + self.state[1] +0.265*np.sin(heading_angle)
            return p_x, p_y
        


    def lidar_callback(self, data:LaserScan):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """

        self.lidar_data=data

    def process_lidar_data(self):
        self.proc_ranges = self.preprocess_lidar(self.lidar_data.ranges)
        
        #Find closest point to LiDAR #TODO this needs work
        #closest_point_index=np.argmin(self.proc_ranges) 
        #Eliminate all points inside 'bubble' (set them to zero) #TODO this needs work
        #self.proc_ranges[closest_point_index-self.bubble_radius:closest_point_index+self.bubble_radius]=0
 
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
        if not len(gap_x_array)==0:   #if the list is empty, just stick to the old target points 
            
            
            if self.add_markers:
                
                gap_line=visualiser.GapMarker(gap_x_array, gap_y_array, 1)
                gap_line.draw_point()
                #a_x, a_y= self.lidar_to_xy(0)
                #gap_point=visualiser.TargetMarker(a_x, a_y, 1)
                gap_point=visualiser.TargetMarker(self.t_x, self.t_y, 1)
                gap_point.draw_point()
            

    def prepare_goal_template(self, t_now):
        if not ('lidar_data' in self.__dict__.keys()):
            rospy.loginfo("No lidar data yet")
        else:
            self.process_lidar_data()
            
        template = self.controller.get_tvp_template()

        for k in range(self.n_horizon + 1):
            
            template["_tvp", k, "target_x"]=self.t_x
            template["_tvp", k, "target_y"] =self.t_y
        
        return template     
  

def main(args):
    
    
    rospy.init_node("mpc_node", anonymous=True)
    rospy.loginfo("starting up mpc node")
    model_predictive_control =FTGController(max_speed=None, max_range=4, bubble_radius=10, threshold=4)
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)





