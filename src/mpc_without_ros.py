# -*- coding: utf-8 -*-
"""
Created on Thu May 26 14:58:32 2022

@author: Fiona
"""

from __future__ import print_function
from importlib.machinery import PathFinder
import sys
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
import os


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
from do_mpc.graphics import Graphics





class BaseController:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        self.wheelbase=0.3302
        self.max_steering_angle=0.4189 # radians
        self.max_accel=7.51 # meters/second^2
        self.max_decel= 8.26 # meters/second^2
        self.max_steering_vel= 3.2 # radians/second
        self.friction_coeff= 0.523 # - (complete estimate)
        self.height_cg= 0.074 # m (roughly measured to be 3.25 in)
        self.l_cg2rear= 0.17145 # m (decently measured to be 6.75 in)
        self.l_cg2front= 0.15875
        self.setup_mpc()
        self.configure_graphics()



    def make_mpc_step(self, x_state):

        u = self.controller.make_step(x_state)
        simulated_x = self.simulator.make_step(u)
        #print("expected new state (x,y,phi) {}".format(simulated_x))

        delta=u[0]
        v=u[1]
        return u, simulated_x




    def setup_mpc(self):
        #print("setting up MPC")
        model_type = 'continuous' # either 'discrete' or 'continuous'
        self.model = Model(model_type)

        #state
        self.x = self.model.set_variable(var_type='_x', var_name='x', shape=(1,1)) #global position x
        self.y =self.model.set_variable(var_type='_x', var_name='y', shape=(1,1)) #global position y
        self.phi = self.model.set_variable(var_type='_x', var_name='phi', shape=(1,1)) #heading of car
        self.delta = self.model.set_variable(var_type='_u', var_name='delta', shape=(1,1))# front steering angle
        self.v = self.model.set_variable(var_type='_u', var_name='v', shape=(1,1)) # velocity

        l_r=self.l_cg2rear
        l_f=self.l_cg2front
        l=l_r+l_f
        beta=self.model.set_expression('beta', np.arctan((l_r/l)*np.tan(self.delta)))
        self.target_x=self.model.set_variable(var_type='_tvp', var_name='target_x', shape=(1,1))
        self.target_y=self.model.set_variable(var_type='_tvp', var_name='target_y', shape=(1,1))

        #differential equations

        # dx_dt= self.v * casadi.cos(self.phi+beta)
        # dy_dt= self.v * casadi.sin(self.phi+beta)
        # dphi_dt=(self.v/l_r)*casadi.sin(beta)

        slip_factor = self.model.set_expression('slip_factor', casadi.arctan(l_r * casadi.tan(self.delta) /self.wheelbase))
        dx_dt= self.v * casadi.cos(self.phi + slip_factor)
        dy_dt= self.v * casadi.sin(self.phi + slip_factor)
        dphi_dt=self.v * casadi.tan(self.delta)* casadi.cos(slip_factor) / self.wheelbase

        self.model.set_rhs('x', dx_dt)
        self.model.set_rhs('y', dy_dt)
        self.model.set_rhs('phi', dphi_dt)

        self.goal_x=30
        self.goal_y=10

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
        self.controller.bounds['lower','_u','delta'] = - self.max_steering_angle
        self.controller.bounds['upper','_u','delta'] = self.max_steering_angle

        self.controller.bounds['lower','_u','v'] = 0 #not going backwards
        self.controller.bounds['upper','_u','v'] = 4#self.params['max_speed']


        #tried to set halfspace constraints but getting this error:
        #CasADi - 2022-06-23 17:10:23 WARNING("S:nlp_g failed: NaN detected for output g, at (row 15, col 0).")CasADi - 2022-06-23 17:10:23 WARNING("S:nlp_g failed: NaN detected for output g, at (row 15, col 0).")CasADi - 2022-06-23 17:10:23 WARNING("S:nlp_g failed: NaN detected for output g, at (row 15, col 0).")
        #also haven't figured out yet what exactly upperbound should be (and if I need to switch the sign on upper_y onn the tvp template)
        
        
        
        #self.controller.set_nl_cons('upper', self.model.tvp['upper_y'], ub=0, soft_constraint=True, penalty_term_cons=1e4)
        #self.controller.set_nl_cons('upper', -self.upper_y, ub = 0)
        #self.controller.set_nl_cons('upper', self.tvp_path_data_y_l+(self.tvp_path_tangent_y/self.tvp_path_tangent_x)*(self.state[0]-self.tvp_path_data_x_l), ub = 0)
        
        self.controller.set_objective(lterm=self.stage_cost, mterm=self.terminal_cost)
        self.r=20
        self.controller.set_rterm(v=self.r)
        self.controller.set_rterm(delta=self.r)

        self.controller.setup()

        x_0 = 0
        y_0 = 0
        phi_0 = 0
        state_0 = np.array([x_0,y_0,phi_0])
        self.controller.x0 = state_0

        # Set initial guess for MHE/MPC based on initial state.
        self.controller.set_initial_guess()

        print("MPC set up finished")

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
    def configure_graphics(self):
        """
        Matplotlib-based plotter and connect relevant data points to it.
        Additional styling is added for more pleasing visuals and can be extended for custom plotting.
        this function was copied from https://github.com/TheCodeSummoner/f1tenth-racing-algorithms
        """
        self._plotter = Graphics(self.controller.data)

        # Add some nice styling
        matplotlib.rcParams["font.size"] = 18
        matplotlib.rcParams["lines.linewidth"] = 3
        matplotlib.rcParams["axes.grid"] = True

        # Create the figure and the axis
        figure, axis = plt.subplots(3, sharex="all", figsize=(16, 9))
        figure.align_ylabels()

        # Draw relevant state and inputs
        self._plotter.add_line(var_type="_x", var_name="x", axis=axis[0], color="green")
        self._plotter.add_line(var_type="_x", var_name="y", axis=axis[0], color="blue")
        self._plotter.add_line(var_type="_x", var_name="phi", axis=axis[1], color="red")
        self._plotter.add_line(var_type="_u", var_name="delta", axis=axis[1], color="green")
        self._plotter.add_line(var_type="_u", var_name="v", axis=axis[2], color="red")

        # Set X and Y labels
        axis[0].set_ylabel("Position")
        axis[1].set_ylabel("Angles")
        axis[2].set_ylabel("Velocity")
        axis[2].set_xlabel("Time")



my_controller =BaseController()

simulated_x=my_controller.controller.x0
n=300
for i in range(n):
    u, simulated_x= my_controller.make_mpc_step(simulated_x)

my_controller._plotter.plot_results()
my_controller._plotter.reset_axes()
plt.show()

data_array=my_controller.controller.data['_x']
x_data=data_array[:,0]
y_data=data_array[:,1]

fig = plt.figure(figsize=(10,5))
plt.plot(x_data, y_data)
plt.xlabel('x position')
plt.ylabel('y position')
title="goal(x,y)=({},{}), r_term={}".format(my_controller.goal_x, my_controller.goal_y, my_controller.r)
plt.title(title)
plt.show()
filepath=os.getcwd()+"/plots/"+title
#fig.savefig(filepath, bbox_inches='tight', dpi=150)





