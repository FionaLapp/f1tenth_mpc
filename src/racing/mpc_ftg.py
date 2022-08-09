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
from std_msgs.msg import String

from rospy.rostime import Duration, Time

import mpc_base_code as mpc_base_code
import helper.visualiser as visualiser

class FTGController(mpc_base_code.BaseController):
    """ 
    """
    def __init__(self, add_markers=True, time_step=0.1):
        self.params=super().get_params()
        self.read_desired_path()
        self.current_steering_angle=0
        self.setup_finished=False
        self.add_markers=add_markers
        self.setup_laser_scan()
        self.t_x=0
        self.t_y=0
        self.previous_delta=0
        self.distance_to_wall_ahead=1000 # a large number
        self.state=[0,0,0]
        self.laps_completed=0
        super().setup_node()
        if self.params['velocity']<= self.params['max_speed']:
            max_speed=self.params['velocity']
        else:
            rospy.loginfo("Can't go that fast, only able to drive {}m/s but you requested {}m/s. I'll drive as fast as I can though :-)".format(self.params['max_speed'], self.params['velocity']))
            max_speed=self.params['max_speed']        
        self.max_range=max_speed*time_step*self.params['n_horizon']#v*t=s, t=delta_t*horizon
        self.threshold=self.max_range
        self.cutoff_per_side=150 #so the target point isn't right behind the car
        self.angle_increment=0.005823155865073204 #can I get this from some param file? probably. would that be cleaner? certainly. Will I bother? Doesn't look likely
        self.raw_scan_angle_min=-3.1415927410125732 #could just say -np.pi, but where'd be the fun in that?
        self.angle_min=self.raw_scan_angle_min+(self.angle_increment*self.cutoff_per_side)
        self.min_gap_width= self.params['width']/(self.angle_increment*self.max_range) #min_gap_width=car_width=angle_inc*min_gap_number*max_range (approximately, for large max_ranges  since that'd give the arclength)
        
        super().setup_mpc(max_speed=max_speed, n_horizon=self.params['n_horizon'], time_step=time_step)
        
        self.key_pub.publish(String("n"))
        self.lap_start_time=Time.now()
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

            self.previous_delta=self.state[2]
            x=data.pose.pose.position.x
            y=data.pose.pose.position.y
            orientation_list=[data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
            (roll, pitch, phi) = euler_from_quaternion (orientation_list)
            #rospy.loginfo("{}, {}, {}".format(x, y, phi))
            distances_to_current_point=(self.path_data_x-self.state[0])**2+(self.path_data_y-self.state[1])**2
            closest=(distances_to_current_point.argmin()+2) #not actually the closest because we want to always be ahead
            self.index= closest %self.path_length
            if np.abs(closest - self.path_length)<10:
                super().on_lap_complete()
            #if np.abs(self.index-self.plot_index)<5:
            if self.previous_x>80:
                print("40")
                self.key_pub.publish(String("p"))
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
        return ranges_convolved

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

    def lidar_to_xy(self, index, ranges):
        if self.lidar_data is None:
            raise Warning("no laser data provided")
        else:
            laser_angle = (index * self.angle_increment) + self.angle_min 
            heading_angle=self.state[2]
            point_angle = laser_angle + heading_angle
            p_x = ranges[index] * np.cos(point_angle) + self.state[0] +0.265*np.cos(heading_angle)
            p_y =ranges[index]  * np.sin(point_angle) + self.state[1] +0.265*np.sin(heading_angle)
            return p_x, p_y
        
    def lidar_to_xy_range(self, start_index, end_index, ranges):
        if self.lidar_data is None:
            raise Warning("no laser data provided")
        else:
            laser_angle = (np.arange(start_index, end_index) * self.angle_increment) + self.angle_min 
            heading_angle=self.state[2]
            point_angle = laser_angle + heading_angle
            p_x = ranges[start_index:end_index] * np.cos(point_angle) + self.state[0] +0.265*np.cos(heading_angle)
            p_y =ranges[start_index:end_index]  * np.sin(point_angle) + self.state[1] +0.265*np.sin(heading_angle)
            return p_x, p_y

    def lidar_callback(self, data:LaserScan):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """

        self.lidar_data=data

    def process_lidar_data(self):
        self.uncut_ranges= self.preprocess_lidar(self.lidar_data.ranges)
        self.cutoff_ranges = np.where(self.uncut_ranges<=self.max_range, self.uncut_ranges, self.threshold) #set everything larger than max_range to max_range
        
        #Find closest point to LiDAR #TODO this needs work
        #closest_point_index=np.argmin(self.cutoff_ranges) 
        #Eliminate all points inside 'bubble' (set them to zero) #TODO this needs work
        #self.cutoff_ranges[closest_point_index-self.bubble_radius:closest_point_index+self.bubble_radius]=0
 
        #Find max length gap 
        start_index, end_index=self.find_max_gap(self.cutoff_ranges)
        

        #Find the best point in the gap 
        best_point_index=self.find_best_point_index(start_index, end_index, self.cutoff_ranges)
        
        
        #calculate x and y value, given the index of the lidar point
        self.t_x, self.t_y=self.lidar_to_xy(best_point_index, self.cutoff_ranges)
        gap_x_array=[]
        gap_y_array=[]
        for i in range(start_index, end_index):
            temp_x, temp_y=self.lidar_to_xy(i, self.cutoff_ranges)
            gap_x_array.append(temp_x)
            gap_y_array.append(temp_y)

        self.wall_ahead_x, self.wall_ahead_y=self.lidar_to_xy(best_point_index, self.uncut_ranges)
        self.distance_to_wall_ahead=(self.state[0]-self.wall_ahead_x)**2+(self.state[1]-self.wall_ahead_y)**2
        #print(self.distance_to_wall_ahead)
        #self.threshold=0.01*self.distance_to_wall_ahead
        points_per_side=int((end_index-start_index)/2)
        avg_wall_ahead_x, avg_wall_ahead_y=self.lidar_to_xy_range(best_point_index-points_per_side, best_point_index+points_per_side+1, self.uncut_ranges)
        avg_distance_to_wall_ahead=(self.state[0]-avg_wall_ahead_x)**2+(self.state[1]-avg_wall_ahead_y)**2
        #print("avg:{}, point:{}".format(np.mean(avg_distance_to_wall_ahead),self.distance_to_wall_ahead))
        if self.distance_to_wall_ahead<=7*self.params['center_to_wall_distance']:
            print("about_to_crash")
        if not len(gap_x_array)==0:   #if the list is empty, just stick to the old target points 
            
            
            if self.add_markers:
                
                gap_line=visualiser.GapMarker(gap_x_array, gap_y_array, 1)
                gap_line.draw_point()
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

            template["_tvp", k, "measured_steering_angle"] =self.current_steering_angle
            template["_tvp", k, "wall_distance"] =self.distance_to_wall_ahead
            
        return template     
    @property
    def stage_cost(self):
        """
        none
        """
        #return (self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2 +13*self.measured_steering_angle*self.v #+(200/self.wall_distance)*self.v
        return (self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2 +(self.params['velocity_weight']/self.wall_distance)*self.v**2 #+(200/self.wall_distance)*self.v
     

def main(args):
    
    
    rospy.init_node("mpc_node", anonymous=True)
    rospy.loginfo("starting up mpc node")
    time_step=0.1
    model_predictive_control =FTGController( time_step=time_step)
    #rospy.Timer(rospy.Duration(30), model_predictive_control.plot_mpc)
    
    #rospy.sleep(time_step)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)





