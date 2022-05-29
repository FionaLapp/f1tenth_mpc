#!/usr/bin/env python3
"""
Created on Thu May 12 12:06:18 2022
@author: Fiona
"""
from __future__ import print_function
from importlib.machinery import PathFinder
import sys
import math
import numpy as np
import pandas as pd


#MPC imports
# Add do_mpc to path. This is not necessary if it was installed via pip.
import sys
sys.path.append('../../')

import matplotlib.pyplot as plt
import casadi

# Import do_mpc package:
from do_mpc.model import Model
from do_mpc.controller import MPC
from do_mpc.simulator import Simulator
from do_mpc.data import MPCData


#ROS Imports
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion

import visualiser

class BaseController:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        self.params=self.get_params()
        self.setup_mpc()
        self.setup_node()
    def setup_node(self):
        #Topics & Subs, Pubs
        localisation_topic= '/trajectory'#change to a different topic if applicable (e.g. if using hector)
        drive_topic = '/nav'
        debug_topic= '/debug'
        path_topic='/goal_path'
        #vis_topic='/vis'
        
        self.localisation_sub=rospy.Subscriber(localisation_topic, Path, self.localisation_callback)
        self.path_sub=rospy.Subscriber(path_topic, Path, self.path_callback)
        
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        self.debug_pub=rospy.Publisher(debug_topic, String, queue_size=1)

        #self.vis_pub=rospy.Publisher(vis_topic, Marker, queue_size=100)


    def get_params(self):
        #I quite possibly came up with the worst way to do this
        rospy.loginfo("Getting params")
        params={}
        namespace='mpc/'
        params['wheelbase']= rospy.get_param(namespace+'wheelbase', 0.3302) # meters
        params['width']= rospy.get_param(namespace+'width', 0.2032) # meters (width of racecar)
        params['buffer_length']= rospy.get_param(namespace+'buffer_length', 5) # steering delay
        params['max_speed']=rospy.get_param(namespace+'max_speed', 7) #  meters/second
        params['max_steering_angle']= rospy.get_param(namespace+'max_steering_angle',  0.4189 )# radians
        params['max_accel']= rospy.get_param(namespace+'max_accel', 7.51) # meters/second^2
        params['max_decel']= rospy.get_param(namespace+'max_decel',  8.26) # meters/second^2
        params['max_steering_vel']= rospy.get_param(namespace+'max_steering_vel',  3.2) # radians/second
        params['friction_coeff']= rospy.get_param(namespace+'friction_coeff', 0.523) # - (complete estimate)
        params['height_cg']= rospy.get_param(namespace+'height_cg', 0.074) # m (roughly measured to be 3.25 in)
        params['l_cg2rear']= rospy.get_param(namespace+'l_cg2rear', 0.17145) # m (decently measured to be 6.75 in)
        params['l_cg2front']= rospy.get_param(namespace+'l_cg2front', 0.15875) # m (decently measured to be 6.25 in)
        params['C_S_front']= rospy.get_param(namespace+'C_S_front', 4.718) #.79 # 1/rad ? (estimated weight/4)
        params['C_S_rear']= rospy.get_param(namespace+'C_S_rear', 5.4562) #.79 # 1/rad ? (estimated weight/4)
        params['mass']= rospy.get_param(namespace+'mass', 3.47) # kg (measured on car 'lidart')
        params['moment_inertia']= rospy.get_param(namespace+'moment_inertia', .04712) # kg m^2 (estimated as a rectangle with width and height of car and evenly distributed mass, then shifted to account for center of mass location)
        params['update_pose_rate']= rospy.get_param(namespace+'update_pose_rate', 0.001) # The rate at which the pose and the lidar publish
        # Lidar simulation parameters
        params['scan_beams']= rospy.get_param(namespace+'scan_beams', 1080)
        params['scan_field_of_view']= rospy.get_param(namespace+'scan_field_of_view', 6.2831853) #4.71 # radians
        # The distance from the center of the rear axis (base_link) to the lidar
        params['scan_distance_to_base_link']= rospy.get_param(namespace+'scan_distance_to_base_link', 0.275) # meters
        return params    

    def make_mpc_step(self, x_state):
        
        u = self.controller.make_step(x_state)
        simulated_x = self.simulator.make_step(u)

        x_pred=self.controller.data.prediction(('_x', 'x'))[0]
        y_pred=self.controller.data.prediction(('_x', 'y'))[0]
        
        vis_point=visualiser.TrajectoryMarker(x_pred, y_pred, 1)  
        vis_point.draw_point()
        #rospy.loginfo("predicted x:{}, y:{}. Actual y: {}".format(x_pred[0][0], y_pred[0][0], x_state))
        
        #rospy.loginfo("expected new state (x,y,phi) {}".format(simulated_x))
        
        delta=u[0]
        v=u[1]

        #setup drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "drive"
        drive_msg.drive.steering_angle = delta
        drive_msg.drive.speed=v
        self.drive_pub.publish(drive_msg)
        

    def localisation_callback(self, data:Path):
        """
        Could be from any source of localisation (e.g. odometry or lidar)
        """
        
        car_state=self.get_state_from_data(data.poses[-1])
        #rospy.loginfo("Actual car position (x,y,phi)={}".format(car_state))
        self.make_mpc_step(car_state)
    
    def path_callback(self, data:Path):
        """
        Update goal position
        """

        for i in range(10):
            goal_x=data.poses[i].pose.position.x
            goal_y=data.poses[i].pose.position.y
            
        self.goal_x=goal_x
        self.goal_y=goal_y
        vis_point=visualiser.TargetMarker(self.goal_x, self.goal_y, 1)
        vis_point.draw_point()
        

    def get_state_from_data(self, data:PoseStamped):
        """
        Takes in PoseStamped, returns array of [x,y,heading_angle]
        """
        x=data.pose.position.x
        y=data.pose.position.y
        orientation_list=[data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
        (roll, pitch, phi) = euler_from_quaternion (orientation_list)
        return np.array([x,y, phi])


    def setup_mpc(self):
        rospy.loginfo("setting up MPC")
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
        beta=self.model.set_expression('beta', np.arctan((l_r/l)*np.tan(self.delta)))
        self.target_x=self.model.set_variable(var_type='_tvp', var_name='target_x', shape=(1,1))
        self.target_y=self.model.set_variable(var_type='_tvp', var_name='target_y', shape=(1,1))
        
        #differential equations

        # dx_dt= self.v * casadi.cos(self.phi+beta)
        # dy_dt= self.v * casadi.sin(self.phi+beta)
        # dphi_dt=(self.v/l_r)*casadi.sin(beta)

        slip_factor = self.model.set_expression('slip_factor', casadi.arctan(l_r * casadi.tan(self.delta) /self.params['wheelbase']))
        dx_dt= self.v * casadi.cos(self.phi + slip_factor)
        dy_dt= self.v * casadi.sin(self.phi + slip_factor)
        dphi_dt=self.v * casadi.tan(self.delta)* casadi.cos(slip_factor) / self.params['wheelbase']

        self.model.set_rhs('x', dx_dt)
        self.model.set_rhs('y', dy_dt)
        self.model.set_rhs('phi', dphi_dt)
        
        self.goal_x=2
        self.goal_y=1

        #setup
        self.model.setup()
        self.controller = MPC(self.model)
        suppress_ipopt = {'ipopt.print_level':0, 'ipopt.sb': 'yes', 'print_time':0}
        self.controller.set_param(nlpsol_opts = suppress_ipopt)
        
        #optimiser parameters
        setup_mpc = {
            'n_horizon': 20,
            't_step': 0.1,
            'n_robust': 1,
            'store_full_solution': True,
        }
        self.controller.set_param(**setup_mpc)
        self.controller.set_tvp_fun(self.prepare_goal_template)
        
        # Constraints on steering angle
        self.controller.bounds['lower','_u','delta'] = - self.params['max_steering_angle']
        self.controller.bounds['upper','_u','delta'] = self.params['max_steering_angle']

        self.controller.bounds['lower','_u','v'] = 0.7 #not going backwards
        self.controller.bounds['upper','_u','v'] = 1#self.params['max_speed']

        self.controller.set_objective(lterm=self.stage_cost, mterm=self.terminal_cost)
        self.controller.set_rterm(v=200)
        self.controller.set_rterm(delta=200)

        self.controller.setup()

        x_0 = 0
        y_0 = 0
        phi_0 = 0
        state_0 = np.array([x_0,y_0,phi_0])
        self.controller.x0 = state_0
       
        # Set initial guess for MHE/MPC based on initial state.
        self.controller.set_initial_guess()
        
        rospy.loginfo("MPC set up finished")

        #setup simulator
        self.simulator = Simulator(self.model)
        self.simulator.set_param(t_step = 0.1)
        self.simulator.set_tvp_fun(self.prepare_goal_template_simulator)
        self.simulator.x0 = state_0
        
        self.simulator.setup()

    def prepare_goal_template_simulator(self, _):
    
        template = self.simulator.get_tvp_template()
        template['target_x']=self.goal_x
        template['target_y']=self.goal_y
            

        return template
    
        
  

    def prepare_goal_template(self, t_now):
        
        template = self.controller.get_tvp_template()

        for k in range(20 + 1):
            
            template["_tvp", k, "target_x"]=self.goal_x
            template["_tvp", k, "target_y"] =self.goal_y
        #rospy.loginfo("template prepared with goal (x,y)= ({}, {})".format(self.goal_x, self.goal_y))    
        return template    

        

   

    @property
    def stage_cost(self):
        """
        none
        """
        return casadi.DM.zeros()

    @property
    def terminal_cost(self):
        """
        difference between target and actual
        """
        
        return (self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2 
        

def main(args):
    
    
    rospy.init_node("mpc_node", anonymous=True)
    rospy.loginfo("starting up mpc node")
    model_predictive_control =BaseController()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)





