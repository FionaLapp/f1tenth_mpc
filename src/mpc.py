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

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class MPC:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)#TODO: Subscribe to LIDAR
        
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)#TODO: Publish to drive
        #self.get_mpc_step()
        self.initialize_mpc()
        

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement
        return 0.0

    def get_mpc_step(self):
        '''
        self.u0 = self.mpc.make_step(x0)
        self.x0 = self.simulator.make_step(u0)
        '''
        delta=0#self.u0[0]
        a=0#self.u0[1]
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = delta
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

        l_r=1
        l_f=1
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
        self.mpc.bounds['lower','_u','delta'] = -0.7
        self.mpc.bounds['upper','_u','delta'] = 0.7

        self.mpc.bounds['lower','_u','a'] = -1
        self.mpc.bounds['upper','_u','a'] = 5


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
        

def main(args):
    
    rospy.init_node("mpc_node", anonymous=True)
    rospy.loginfo("starting up mpc node")
    mpc = MPC()
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

