#!/usr/bin/env python3
"""
Created on Thu May 12 12:06:18 2022
@author: Fiona
"""
from __future__ import print_function
from cgi import print_directory
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib


#MPC imports
# Add do_mpc to path. This is not necessary if it was installed via pip.
import sys
sys.path.append('../../')

import casadi

# Import do_mpc package:
from do_mpc.model import Model
from do_mpc.controller import MPC
from do_mpc.graphics import Graphics

#ROS Imports
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

import mpc_base_code as mpc_base_code
import helper.visualiser as visualiser
from mpc_centerline_with_constraints import ControllerWithConstraints as constraint_base_class
from mpc_ftg import FTGController as lidar_base_class
class ControllerWithLidarConstraints(constraint_base_class):
    """ This controoller reads in the line data from the centerline file, 
    then callculates wall points for each centerline point using the track-with provided in the custom param file
    """
    def __init__(self, max_speed=None, add_markers=True, max_range=3, bubble_radius=10, threshold=3):
        self.setup_finished=False
        self.add_markers=add_markers
        self.params=super().get_params()
        super().read_desired_path()
        self.setup_node()
        if max_speed is None:
            max_speed=self.params['max_speed']
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
        self.lidar_data=None
        self.lidar_constraint_x_l=0
        self.lidar_constraint_x_r=0
        self.lidar_constraint_y_l=0
        self.lidar_constraint_y_r=0
        self.setup_mpc(max_speed=max_speed)
        
        self.setup_finished=True
        #super()().__init__(max_speed, add_markers)  
    
    def setup_node(self):
        localisation_topic= '/odom' #change to a different topic if applicable (e.g. if using hector)
        drive_topic = '/nav'
        laser_topic= '/scan'


        self.localisation_sub=rospy.Subscriber(localisation_topic, Odometry, self.localisation_callback)
        self.fixing_a_weird_bug_and_not_much_else_sub=rospy.Subscriber('/odom', Odometry, self.pose_callback, queue_size=1)#subscribing to /odom a second time somehow makes the first one work, otherwise it gets stuck at the origin
    
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        
        laser_sub=rospy.Subscriber(laser_topic, LaserScan, self.lidar_callback)

    def localisation_callback(self, data:Odometry):
        #rospy.loginfo("localisation from mpc_constraint_with_lidar")
    
        if self.lidar_data is None:
            rospy.loginfo("No lidar data yet")
            return
        self.process_lidar_data()
        super().localisation_callback(data)
        


    
    
    # def preprocess_lidar(self, ranges):
        
    #     return lidar_base_class.preprocess_lidar(self, ranges)
    #     #print(self.lidar_data.ranges)

    # def find_max_gap(self, ranges):
    #     lidar_base_class.find_max_gap(self, ranges)
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
            rospy.logdebug("no walls found near car, will continue going straight")
            return 0, len(ranges)
        index_in_gap_index_array=np.argmax(gap_size)#find the index of the largest gap (this index refers to the gap_index_array)
        start_index=gap_index_array[index_in_gap_index_array]+1 #the gap starts one index after the non-gap
        end_index=gap_index_array[index_in_gap_index_array+1] # the gap ends at the next non-gap
        return start_index, end_index

    def lidar_to_xy(self, index):
        if self.lidar_data is None:
            raise Warning("no laser data provided")
        else:
            laser_angle = (index * self.angle_increment) + self.angle_min 
            heading_angle=self.state[2]
            point_angle = laser_angle + heading_angle
            p_x = self.proc_ranges[index] * np.cos(point_angle) + self.state[0] +0.265*np.cos(heading_angle)
            p_y =self.proc_ranges[index]  * np.sin(point_angle) + self.state[1]+0.265*np.sin(heading_angle)
            return p_x, p_y
        
    

    # def lidar_to_xy(self, index):
    #     lidar_base_class.lidar_to_xy(self, index)
        


    def lidar_callback(self, data:LaserScan):
        lidar_base_class.lidar_callback(self=self, data=data)

    def process_lidar_data(self):
        rospy.logdebug("processing lidar data from mpc_constraints_with_lidar")
        self.proc_ranges = self.preprocess_lidar(self.lidar_data.ranges)
        
        #Find closest point to LiDAR #TODO this needs work
        #closest_point_index=np.argmin(self.proc_ranges) 
        #Eliminate all points inside 'bubble' (set them to zero) #TODO this needs work
        #self.proc_ranges[closest_point_index-self.bubble_radius:closest_point_index+self.bubble_radius]=0
 
        #Find max length gap 
        start_index, end_index=self.find_max_gap(self.proc_ranges)
        
        #calculate x and y value, given the index of the lidar point
        self.lidar_constraint_x_l, self.lidar_constraint_y_l=self.lidar_to_xy(start_index)
        self.lidar_constraint_x_r, self.lidar_constraint_y_r=self.lidar_to_xy(end_index)
        if self.add_markers:
            gap_x_array=[]
            gap_y_array=[]
            for i in range(start_index, end_index):
                temp_x, temp_y=self.lidar_to_xy(i)
                gap_x_array.append(temp_x)
                gap_y_array.append(temp_y)
            if not len(gap_x_array)==0:   #if the list is empty, just stick to the old target points 
                gap_line=visualiser.GapMarker(gap_x_array, gap_y_array, 1)
                gap_line.draw_point()
                
    def setup_mpc(self, max_speed, n_horizon=5):
        rospy.loginfo("setting up MPC from child class")
        model_type = 'continuous' # either 'discrete' or 'continuous'
        self.model = Model(model_type)

        #state
        self.x = self.model.set_variable(var_type='_x', var_name='x', shape=(1,1)) #global position x
        self.y =self.model.set_variable(var_type='_x', var_name='y', shape=(1,1)) #global position y
        self.phi = self.model.set_variable(var_type='_x', var_name='phi', shape=(1,1)) #heading of car
        self.delta = self.model.set_variable(var_type='_u', var_name='delta', shape=(1,1))# front steering angle
        self.v = self.model.set_variable(var_type='_u', var_name='v', shape=(1,1)) # velocity
        
        l_r=self.params['l_cg2rear']
        l_f=self.params['l_cg2front']
        l=l_r+l_f
        self.target_x=self.model.set_variable(var_type='_tvp', var_name='target_x', shape=(1,1))
        self.target_y=self.model.set_variable(var_type='_tvp', var_name='target_y', shape=(1,1))
        
        ##constraint params
        ##
        #self.upper_x=self.model.set_variable(var_type='_tvp', var_name='upper_x', shape=(1,1))
        self.constraint_t_x=self.model.set_variable(var_type='_tvp', var_name='constraint_t_x', shape=(1,1))
        self.constraint_t_y=self.model.set_variable(var_type='_tvp', var_name='constraint_t_y', shape=(1,1))
        
        self.constraint_p_x_lower=self.model.set_variable(var_type='_tvp', var_name='constraint_p_x_lower', shape=(1,1))
        self.constraint_p_x_upper=self.model.set_variable(var_type='_tvp', var_name='constraint_p_x_upper', shape=(1,1))
        self.constraint_p_y_lower=self.model.set_variable(var_type='_tvp', var_name='constraint_p_y_lower', shape=(1,1))
        self.constraint_p_y_upper=self.model.set_variable(var_type='_tvp', var_name='constraint_p_y_upper', shape=(1,1))
        
        self.constraint_p_x_lower_lidar=self.model.set_variable(var_type='_tvp', var_name='constraint_p_x_lower_lidar', shape=(1,1))
        self.constraint_p_x_upper_lidar=self.model.set_variable(var_type='_tvp', var_name='constraint_p_x_upper_lidar', shape=(1,1))
        self.constraint_p_y_lower_lidar=self.model.set_variable(var_type='_tvp', var_name='constraint_p_y_lower_lidar', shape=(1,1))
        self.constraint_p_y_upper_lidar=self.model.set_variable(var_type='_tvp', var_name='constraint_p_y_upper_lidar', shape=(1,1))
        
        #self.lower_x=self.model.set_variable(var_type='_tvp', var_name='lower_x', shape=(1,1))
        #self.lower_y=self.model.set_variable(var_type='_tvp', var_name='lower_y', shape=(1,1))

        # self.tvp_path_data_y_l=self.model.set_variable(var_type="_tvp", var_name="tvp_path_data_y_l", shape=(1,1))
        # self.tvp_path_data_x_l=self.model.set_variable(var_type="_tvp", var_name="tvp_path_data_x_l", shape=(1,1))
        # self.tvp_path_tangent_y=self.model.set_variable(var_type="_tvp", var_name="tvp_path_tangent_y", shape=(1,1))
        # self.tvp_path_tangent_x=self.model.set_variable(var_type="_tvp", var_name="tvp_path_tangent_x", shape=(1,1))
        
            

        #differential equations

        slip_factor = self.model.set_expression('slip_factor', casadi.arctan(l_r * casadi.tan(self.delta) /self.params['wheelbase']))
        dx_dt= self.v * casadi.cos(self.phi + slip_factor)
        dy_dt= self.v * casadi.sin(self.phi + slip_factor)
        dphi_dt=self.v * casadi.tan(self.delta)* casadi.cos(slip_factor) / self.params['wheelbase']

        self.model.set_rhs('x', dx_dt)
        self.model.set_rhs('y', dy_dt)
        self.model.set_rhs('phi', dphi_dt)
        
        self.goal_x=0
        self.goal_y=0

        #setup
        self.model.setup()
        self.controller = MPC(self.model)
        suppress_ipopt = {'ipopt.print_level':0, 'ipopt.sb': 'yes', 'print_time':0}
        self.controller.set_param(nlpsol_opts = suppress_ipopt)
        self.n_horizon=n_horizon
        #optimiser parameters
        setup_mpc = {
            'n_horizon': self.n_horizon,
            't_step': 0.1,
            'n_robust': 1,
            'store_full_solution': True,
        }
        self.controller.set_param(**setup_mpc)
        self.controller.set_tvp_fun(self.prepare_goal_template)
        
        # Constraints on steering angle
        self.controller.bounds['lower','_u','delta'] = - self.params['max_steering_angle']
        self.controller.bounds['upper','_u','delta'] = self.params['max_steering_angle']

        self.controller.bounds['lower','_u','v'] = 0 #not going backwards
        self.controller.bounds['upper','_u','v'] = max_speed
        
        self.upper_wall=self.controller.set_nl_cons('upper_wall', self.constraint_t_x*(self.y-self.constraint_p_y_upper)-self.constraint_t_y*(self.x-self.constraint_p_x_upper), ub=0,soft_constraint=True, penalty_term_cons=1e4)
        self.lower_wall=self.controller.set_nl_cons('lower_wall', -(self.constraint_t_x*(self.y-self.constraint_p_y_lower)-self.constraint_t_y*(self.x-self.constraint_p_x_lower)), ub=0, soft_constraint=True, penalty_term_cons=1e4)
        
        self.upper_lidar=self.controller.set_nl_cons('upper_lidar', self.constraint_t_x*(self.y-self.constraint_p_y_upper_lidar)-self.constraint_t_y*(self.x-self.constraint_p_x_upper_lidar), ub=0,soft_constraint=True, penalty_term_cons=1e4)
        self.lower_lidar=self.controller.set_nl_cons('lower_lidar', -(self.constraint_t_x*(self.y-self.constraint_p_y_lower_lidar)-self.constraint_t_y*(self.x-self.constraint_p_x_lower_lidar)), ub=0, soft_constraint=True, penalty_term_cons=1e4)
        

        self.controller.set_objective(lterm=self.stage_cost, mterm=self.terminal_cost)
        self.controller.set_rterm(v=1)
        self.controller.set_rterm(delta=4)

        self.controller.setup()

        x_0 = 0
        y_0 = 0
        phi_0 = 0
        state_0 = np.array([x_0,y_0,phi_0])
        self.controller.x0 = state_0
        self.state=state_0
       
        # Set initial guess for MHE/MPC based on initial state.
        self.controller.set_initial_guess()
        
        rospy.loginfo("MPC set up finished")  
            
   
    def prepare_goal_template(self, t_now):
        template = self.controller.get_tvp_template()
        x_line_list=[]
        y_line_list=[]
        target_marker_list_x=[]
        target_marker_list_y=[]
        for k in range(self.n_horizon + 1):
            i=(self.index+k)%self.path_length
            template["_tvp", k, "target_x"]=self.path_data[' x_m'][i]
            template["_tvp", k, "target_y"] =self.path_data[' y_m'][i]
            #vector equation of line
            #r=p+lambda*t (r: any point on line, p: knownn point on line, lambda: param, t: tangent to line)
            #i.e.
            
            #r_x=p_x+lambda*t_x
            #r_y=p_y+lambda*t_y
            #eliminate lambda from system
            #lambda=(r_x-p_x)/t_x
            #0=r_y-p_y-t_y*(r_x-p_x)/t_x
            # let r_y= self.y, r_x=self.x then the car hits the border
            #hence thee constraint line is 0=y-mx+n
            #where m=p_y-(t_y/t_x))*p_x, n=t_y/t_x
            #but of course this will inevitably cause problems once t_x=0, so we rearrange again:
            #0=t_x*(r_y-p_y)-t_y*(r_x-p_x)
            
            #in a very basic approach, I'm going to just assume the constraint lines are parallel. 
            #I'm then going to figure out which n (left or right) belongs to the upper and which to lower
            #bound by (get ready for it)  comparing them and deducing, with all the two-and-a-half braincells 
            #I have left, that the larger one belongs to the upper bound. Magic.
            template["_tvp", k, "constraint_t_x"] = self.path_tangent_x[i]
            template["_tvp", k, "constraint_t_y"] = self.path_tangent_y[i]
            if self.path_tangent_x[i]==0:
                print("x tangent=0")

                if self.path_tangent_y[i]==0:
                    raise Exception("Tangent vector to wall is 0 vector")
                y_l=self.path_data_y_l[i]
                y_r=self.path_data_y_r[i]
                
                if y_l>y_r:
                    
                    template["_tvp", k, "constraint_p_y_upper"] = self.path_data_y_l[i]
                    template["_tvp", k, "constraint_p_y_lower"] = self.path_data_y_r[i]
                    template["_tvp", k, "constraint_p_y_upper_lidar"] = self.path_data_y_l[i]
                    template["_tvp", k, "constraint_p_y_lower_lidar"] = self.path_data_y_r[i]
                else:
                    template["_tvp", k, "constraint_p_y_upper"] = self.path_data_y_r[i]
                    template["_tvp", k, "constraint_p_y_lower"] = self.path_data_y_l[i]
                    template["_tvp", k, "constraint_p_y_upper_lidar"] = self.path_data_y_r[i]
                    template["_tvp", k, "constraint_p_y_lower_lidar"] = self.path_data_y_l[i]
                
            else:
                
                #calculate y-value of x-value corresponding to current car position. if current car y < line y --> point below line
                n_left=(self.path_data_y_l[i]-(self.path_tangent_y[i]/self.path_tangent_x[i])*self.path_data_x_l[i])
                n_right=(self.path_data_y_r[i]-(self.path_tangent_y[i]/self.path_tangent_x[i])*self.path_data_x_r[i])
                if n_left>n_right:
                    #print("n_left:{}, n_right:{}".format(n_left, n_right))
                    #print("left wall above car")
                    p_x_upper= self.path_data_x_l[i]
                    p_x_lower = self.path_data_x_r[i]
                    p_y_upper = self.path_data_y_l[i]
                    p_y_lower = self.path_data_y_r[i]
                    p_x_upper_lidar= self.lidar_constraint_x_l
                    p_x_lower_lidar = self.lidar_constraint_x_r
                    p_y_upper_lidar = self.lidar_constraint_y_l
                    p_y_lower_lidar = self.lidar_constraint_y_r


                elif n_left<n_right:
                    #print("right wall above car")
                    #print("n_left:{}, n_right:{}".format(n_left, n_right))
                    p_x_upper= self.path_data_x_r[i]
                    p_x_lower= self.path_data_x_l[i]
                    p_y_upper= self.path_data_y_r[i]
                    p_y_lower = self.path_data_y_l[i]
                    p_x_upper_lidar= self.lidar_constraint_x_r
                    p_x_lower_lidar = self.lidar_constraint_x_l
                    p_y_upper_lidar = self.lidar_constraint_y_r
                    p_y_lower_lidar = self.lidar_constraint_y_l
                else: #this should never happen because then the lines are identical
                    raise Exception("n_left=n_right")
                template["_tvp", k, "constraint_p_x_upper"] = p_x_upper
                template["_tvp", k, "constraint_p_x_lower"] = p_x_lower
                template["_tvp", k, "constraint_p_y_upper"] = p_y_upper
                template["_tvp", k, "constraint_p_y_lower"] = p_y_lower
                
                template["_tvp", k, "constraint_p_x_upper_lidar"] = p_x_upper_lidar
                template["_tvp", k, "constraint_p_x_lower_lidar"] = p_x_lower_lidar
                template["_tvp", k, "constraint_p_y_upper_lidar"] = p_y_upper_lidar
                template["_tvp", k, "constraint_p_y_lower_lidar"] = p_y_lower_lidar
           
            

            if self.add_markers:
                
                target_marker_list_x.append(self.path_data_x[i])
                
                target_marker_list_y.append( self.path_data_y[i])
                #plotting lines:
                factor=5 #completely random length factor for displayed line

                #track
                # x_line_list.extend([p_x_upper-factor*self.path_tangent_x[i], p_x_upper+factor*self.path_tangent_x[i]])
                # x_line_list.extend( [p_x_lower-factor*self.path_tangent_x[i], p_x_lower+factor*self.path_tangent_x[i]])
                # y_line_list.extend([p_y_upper-factor*self.path_tangent_y[i], p_y_upper+factor*self.path_tangent_y[i]])
                # y_line_list.extend( [p_y_lower-factor*self.path_tangent_y[i], p_y_lower+factor*self.path_tangent_y[i]])
                
                #lidar data
                x_line_list.extend([p_x_upper_lidar-factor*self.path_tangent_x[i], p_x_upper_lidar+factor*self.path_tangent_x[i]])
                x_line_list.extend( [p_x_lower_lidar-factor*self.path_tangent_x[i], p_x_lower_lidar+factor*self.path_tangent_x[i]])
                y_line_list.extend([p_y_upper_lidar-factor*self.path_tangent_y[i], p_y_upper_lidar+factor*self.path_tangent_y[i]])
                y_line_list.extend([p_y_lower_lidar-factor*self.path_tangent_y[i], p_y_lower_lidar+factor*self.path_tangent_y[i]])
        
        if self.add_markers:
            vis_point=visualiser.TargetMarker(target_marker_list_x, target_marker_list_y, 1)
            vis_point.draw_point()
            constraint_left_marker=visualiser.ConstraintMarker(x_line_list, y_line_list , 1)
            constraint_left_marker.draw_point()

        return template   

    
  
def main(args):
    rospy.init_node("mpc_node", anonymous=True)
    rospy.loginfo("starting up mpc node")
    model_predictive_control =ControllerWithLidarConstraints(max_speed=2, add_markers=True)
    rospy.sleep(0.1)
    rospy.spin()
    
    




if __name__=='__main__':
	main(sys.argv)





