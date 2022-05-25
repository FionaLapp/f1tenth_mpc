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
from casadi import *

# Import do_mpc package:
from do_mpc.model import Model
from do_mpc.controller import MPC
from tf.transformations import euler_from_quaternion


#ROS Imports
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker



class MPC:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        self.params=self.get_params()
        self.setup_mpc()
        self.setup_node()
    def setup_node(self):
        #Topics & Subs, Pubs
        odometry_topic= '/trajectory'#'/odom'#'slam_out_pose'
        drive_topic = '/nav'
        debug_topic= '/debug'
        arrow_topic='/velocity'
        path_topic='/goal_path'
        

        
        self.dummy_velocity=1
        self.goal=[10,1]
        self.arrow_pub=rospy.Publisher(arrow_topic, Marker, queue_size=1)
        #self.visualise_goal(self.goal)

        #self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)#TODO: Subscribe to LIDAR
        self.odom_sub=rospy.Subscriber(odometry_topic, Path, self.odometry_callback)#TODO: Subscribe to LIDAR
        self.path_sub=rospy.Subscriber(path_topic, Path, self.path_callback)#TODO: Subscribe to LIDAR
        
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)#TODO: Publish to drive
        self.debug_pub=rospy.Publisher(debug_topic, String, queue_size=1)


    def visualise_arrow(self, point):
        arrow_msg=Marker()
        arrow_msg.header.stamp = rospy.Time.now()
        arrow_msg.header.frame_id = "base_link"
        arrow_msg.type=arrow_msg.ARROW
        arrow_msg.action = arrow_msg.ADD
        arrow_msg.pose.orientation.z = point[0]
        arrow_msg.pose.orientation.w = point[1]
        arrow_msg.scale.x = 10

        #arrow_msg.lifetime=0
        self.arrow_pub.publish(arrow_msg)
        #self.debug_pub.publish("Published goal")

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

    def get_mpc_step(self, x_state):
        
        u = self.controller.make_step(x_state)
        #x = self.simulator.make_step(u0) #TODO add real measurement here
        
        delta=u[0]
        v=u[1]
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = delta
        drive_msg.drive.speed=v
        #drive_msg.drive.acceleration = a
        #debug_msg=String()
        #debug_msg.data=rospy.get_param()
        #self.debug_pub.publish(debug_msg)
        self.drive_pub.publish(drive_msg)
        #arrow= [a*np.cos(delta), a*np.sin(delta)]
        #self.visualise_arrow(arrow)
        
        #self.visualise_goal(x_state)

    

    def odometry_callback(self, data:Path):
        """
        """
        
        car_state=self.get_state_from_data(data.poses[-1])
        #rospy.loginfo(car_state)
        #print(car_state)
        self.get_mpc_step(car_state)
    
    def path_callback(self, data:Path):
       
        
        for i in range(10):
            goal_x=data.poses[i].pose.position.x
            goal_y=data.poses[i].pose.position.y
            
        #rospy.loginfo(self.goal_array)
        self.goal_x=goal_x
        self.goal_y=goal_y
        rospy.loginfo(self.goal_x)


    def get_state_from_data(self, data:PoseStamped):
        #print(data)
        x=data.pose.position.x
        y=data.pose.position.y
        #v= self.dummy_velocity
        orientation_list=[data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
        (roll, pitch, phi) = euler_from_quaternion (orientation_list)
        #print(euler_from_quaternion (orientation_list))
        #phi=2*np.arcsin(data.pose.orientation.z)

        #also_phi=2*np.arccos(data.pose.pose.position.w)
        #self.debug_pub.publish("phi")
        #self.debug_pub.publish(String(data.pose.pose.position))
        #self.debug_pub.publish("")
        
        return np.array([x,y, phi])


    def setup_mpc(self):
        rospy.loginfo("setting up MPC")
        model_type = 'continuous' # either 'discrete' or 'continuous'
        self.model = Model(model_type)

        #state
        self.x = self.model.set_variable(var_type='_x', var_name='x', shape=(1,1)) #global position x
        self.y =self.model.set_variable(var_type='_x', var_name='y', shape=(1,1)) #global position y
        phi = self.model.set_variable(var_type='_x', var_name='phi', shape=(1,1)) #heading of car
        delta = self.model.set_variable(var_type='_u', var_name='delta', shape=(1,1))# front steering angle
        self.v = self.model.set_variable(var_type='_u', var_name='v', shape=(1,1)) # velocity
        
        #a = self.model.set_variable(var_type='_u', var_name='a', shape=(1,1))# accelleration

        #delta rear=0 since rear cannot be steered

        # states for the desired input:
        # d = model.set_variable(var_type='_u', var_name='d') #duty cycle to drive train
        # delta = model.set_variable(var_type='_u', var_name='delta')  #steering angle
        # v_theta = model.set_variable(var_type='_u', var_name='v_theta') #velocity along reference path


        '''
        Next we define parameters. Known values can and should be hardcoded but with robust MPC in mind, we define uncertain parameters explictly. We assume that the inertia is such an uncertain parameter and hardcode the spring constant and friction coefficient.
        '''

        l_r=self.params['l_cg2rear']
        l_f=self.params['l_cg2front']
        l=l_r+l_f
        beta=self.model.set_expression('beta', np.arctan((l_r/l)*np.tan(delta)))
        self.target_x=self.model.set_variable(var_type='_tvp', var_name='target_x', shape=(1,1))
        self.target_y=self.model.set_variable(var_type='_tvp', var_name='target_y', shape=(1,1))
        
        #differential equation
        dx_dt= self.v * np.cos(phi+beta)
        dy_dt= self.v * np.sin(phi+beta)
        dphi_dt=(self.v/l_r)*sin(beta)
        #dv_dt=a#casadi.SX.zeros(1,1) # a=0
        #ddelta_dt=casadi.SX.zeros(1,1)#omega

        self.model.set_rhs('x', dx_dt)
        self.model.set_rhs('y', dy_dt)
        self.model.set_rhs('phi', dphi_dt)
        #self.model.set_rhs('v', dv_dt)

        self.goal_x=0
        self.goal_y=0
        #self.goal_y=np.zeros(21)
        

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
        
        #objective function
        #TODO
        #x_f   = self.goal[0]
        #y_f   = self.goal[1]
        #v_f   =  0
        #psi_f = -np.pi/2

        # weights for objective function
        w_pos = 100
        w_vel = 0.2

        #lterm = w_pos*((x-x_f)**2 + (y-y_f)**2) + w_vel*(v-v_f)**2
        #mterm = w_pos*((x-x_f)**2 + (y-y_f)**2) + w_vel*(v-v_f)**2

        #self.controller.set_objective(mterm=mterm, lterm=lterm)

        #self.controller.set_rterm(delta=10.)

        #self.controller.set_objective(mterm=mterm, lterm=lterm)

        # Constraints on steering angle
        self.controller.bounds['lower','_u','delta'] = - self.params['max_steering_angle']
        self.controller.bounds['upper','_u','delta'] = self.params['max_steering_angle']

        #self.controller.bounds['lower','_u','a'] = - self.params['max_decel']
        #self.controller.bounds['upper','_u','a'] = self.params['max_accel']

        self.controller.bounds['lower','_u','v'] = 0 #not going backwards
        self.controller.bounds['upper','_u','v'] = self.params['max_speed']

        self.controller.set_objective(lterm=self.stage_cost, mterm=self.terminal_cost)
        self.controller.set_rterm(v=1)
        self.controller.set_rterm(delta=1)

        self.controller.setup()
        #self.simulator = do_mpc.simulator.Simulator(self.model)
        #self.simulator.set_param(t_step = 0.1)
        #self.simulator.set_tvp_fun(self.prepare_goal_template_simulator)
        
        #self.simulator.setup()

        x_0 = 0
        y_0 = 0
        v_0 = 0
        phi_0 = 0
        delta_0=0

        state_0 = np.array([x_0,
                            y_0,
                            
                            phi_0
                            ])

        self.controller.x0 = state_0
        #self.simulator.x0 = state_0
        



        # Set initial guess for MHE/MPC based on initial state.
        self.controller.set_initial_guess()
        
        #mpc_graphics = do_mpc.graphics.Graphics(self.controller.data)
        #sim_graphics = do_mpc.graphics.Graphics(self.simulator.data)
        
        self.u0 = np.zeros((2,1))
        #self.x0=self.simulator.x0
        rospy.loginfo("MPC set up finished")
    
        
    def get_prediction_coordinates(self):
        pathfile_name=rospy.get_param('/path_node/directory')+'/src/maps/Sochi/Sochi_raceline.csv'
        path_data=pd.read_csv(pathfile_name)
        return path_data[' x_m', ' y_m']

    def prepare_goal_template(self, t_now):
        
        #tvp_template = self.controller.get_tvp_template()
        #rospy.loginfo(template)
        #for k in range(20+1):
        #        tvp_template["_tvp",k,'target_x'] = target_array[0, k]
        #        tvp_template["_tvp",k,'target_y'] = target_array[1,k]
        template = self.controller.get_tvp_template()

        for k in range(20 + 1):
            #rospy.loginfo(self.goal_array[0, 19])
            template["_tvp", k, "target_x"]=self.goal_x#"tar#t_now#target_array[0, k]
            template["_tvp", k, "target_y"] =self.goal_y#0#target_array[1,k]
            #rospy.loginfo(target_array)
        #print(template)
        #print((self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2)

        return template    

        

    # def prepare_goal_template_simulator(self, _):
        
    #     template = self.simulator.get_tvp_template()
    #     template['target_x']=10
    #     template['target_y']=20
    #     #template["_tvp", 0, "target_x"] = self.goal_array[0,0]
    #     #template["_tvp", 0, "target_y"] = self.goal_array[1,0]
            

    #     return template

   

    @property
    def stage_cost(self):
        """
        none
        """
        return DM.zeros()

    @property
    def terminal_cost(self):
        """
        difference between target and actual
        """
        
        return (self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2 
        

def main(args):
    
    
    rospy.init_node("mpc_node", anonymous=True)
    rospy.loginfo("starting up mpc node")
    model_predictive_control = MPC()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)






'''

for k in range(10):
  u0 = mpc.make_step(x0)
  x0 = simulator.make_step(u0)

fig, ax, graphics = do_mpc.graphics.default_plot(mpc.data, figsize=(16,9))
graphics.plot_results()
graphics.reset_axes()
plt.show()


fig, ax = plt.subplots(3, figsize=(16,9))


sim_graphics.add_line(var_type='_x', var_name='x', axis=ax[0])
sim_graphics.add_line(var_type='_x', var_name='y', axis=ax[0])


mpc_graphics.add_line(var_type='_u', var_name='a', axis=ax[1])


mpc_graphics.add_line(var_type='_u', var_name='delta', axis=ax[2])
'''

