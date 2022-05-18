#!/usr/bin/env python3
"""
Created on Thu May 12 12:06:18 2022
@author: Fiona
"""
from __future__ import print_function
import sys
import math
import numpy as np


#MPC imports
# Add do_mpc to path. This is not necessary if it was installed via pip.
import sys
sys.path.append('../../')

import matplotlib.pyplot as plt
from casadi import *

# Import do_mpc package:
import do_mpc


#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive




class MPC:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        self.params=self.get_params()
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)#TODO: Subscribe to LIDAR
        
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)#TODO: Publish to drive
        #self.get_mpc_step()
        self.initialize_mpc()

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

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement
        return 0.0

    def get_mpc_step(self):
        
        self.u0 = self.mpc.make_step(self.x0)
        self.x0 = self.simulator.make_step(self.u0)
        
        delta=self.u0[0]
        a=self.u0[1]
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = delta
        drive_msg.drive.speed=1
        drive_msg.drive.acceleration = a
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm
        #TODO:implement
        return 0.0

    def lidar_callback(self, data):
        """
        """
        error = 0.0 #TODO: replace with error returned by followLeft
        #send error to pid_control
        #self.pid_control(error, VELOCITY)
        self.get_mpc_step()

    def initialize_mpc(self):
        rospy.loginfo("Initialising  MPC")
        model_type = 'discrete' # either 'discrete' or 'continuous'
        self.model = do_mpc.model.Model(model_type)

        #state
        x = self.model.set_variable(var_type='_x', var_name='x', shape=(1,1)) #global position x
        y =self.model.set_variable(var_type='_x', var_name='y', shape=(1,1)) #global position y
        v = self.model.set_variable(var_type='_x', var_name='v', shape=(1,1)) # velocity
        phi = self.model.set_variable(var_type='_x', var_name='phi', shape=(1,1)) #heading of car
        delta = self.model.set_variable(var_type='_u', var_name='delta', shape=(1,1))# front steering angle
        a = self.model.set_variable(var_type='_u', var_name='a', shape=(1,1))# accelleration

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

        #differential equation
        dx_dt= v * np.cos(phi+beta)
        dy_dt= v * np.sin(phi+beta)
        dphi_dt=(v/l_r)*sin(beta)
        dv_dt=a#casadi.SX.zeros(1,1) # a=0
        #ddelta_dt=casadi.SX.zeros(1,1)#omega

        self.model.set_rhs('x', dx_dt)
        self.model.set_rhs('y', dy_dt)
        self.model.set_rhs('phi', dphi_dt)
        self.model.set_rhs('v', dv_dt)



        #setup
        self.model.setup()
        self.mpc = do_mpc.controller.MPC(self.model)

        #optimiser parameters
        setup_mpc = {
            'n_horizon': 20,
            't_step': 0.1,
            'n_robust': 1,
            'store_full_solution': True,
        }
        self.mpc.set_param(**setup_mpc)

        #objective function
        #TODO
        x_f   = 50
        y_f   = 20
        v_f   =  0
        psi_f = -np.pi/2

        # weights for objective function
        w_pos = 100
        w_vel = 0.2

        lterm = w_pos*((x-x_f)**2 + (y-y_f)**2) + w_vel*(v-v_f)**2
        mterm = w_pos*((x-x_f)**2 + (y-y_f)**2) + w_vel*(v-v_f)**2

        self.mpc.set_objective(mterm=mterm, lterm=lterm)

        self.mpc.set_rterm(delta=10.)

        self.mpc.set_objective(mterm=mterm, lterm=lterm)

        # Constraints on steering angle
        self.mpc.bounds['lower','_u','delta'] = - self.params['max_steering_angle']
        self.mpc.bounds['upper','_u','delta'] = self.params['max_steering_angle']

        self.mpc.bounds['lower','_u','a'] = - self.params['max_decel']
        self.mpc.bounds['upper','_u','a'] = self.params['max_accel']

        self.mpc.bounds['lower','_x','v'] = 0 #not going backwards
        self.mpc.bounds['upper','_x','v'] = self.params['max_speed']

        self.mpc.setup()
        self.simulator = do_mpc.simulator.Simulator(self.model)
        self.simulator.set_param(t_step = 0.1)
        self.simulator.setup()

        x_0 = 0
        y_0 = 0
        v_0 = 0
        phi_0 = np.pi/4
        delta_0=0

        state_0 = np.array([x_0,
                            y_0,
                            v_0,
                            phi_0
                            ])

        self.mpc.x0 = state_0
        self.simulator.x0 = state_0
        



        # Set initial guess for MHE/MPC based on initial state.
        self.mpc.set_initial_guess()

        mpc_graphics = do_mpc.graphics.Graphics(self.mpc.data)
        sim_graphics = do_mpc.graphics.Graphics(self.simulator.data)


        self.u0 = np.zeros((2,1))
        self.x0=self.simulator.x0
        rospy.loginfo("MPC set up finished")
        delta=1
        a=1
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = delta
        drive_msg.drive.acceleration = a
        self.drive_pub.publish(drive_msg)

        

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

