# -*- coding: utf-8 -*-
"""
Created on Thu May 12 12:06:18 2022

@author: Fiona
"""

import numpy as np

# Add do_mpc to path. This is not necessary if it was installed via pip.
import sys
sys.path.append('../../')

import matplotlib.pyplot as plt
from casadi import *

# Import do_mpc package:
import do_mpc

model_type = 'discrete' # either 'discrete' or 'continuous'
model = do_mpc.model.Model(model_type)

#state
x = model.set_variable(var_type='_x', var_name='x', shape=(1,1)) #global position x
y = model.set_variable(var_type='_x', var_name='y', shape=(1,1)) #global position y
v = model.set_variable(var_type='_x', var_name='v', shape=(1,1)) # velocity
phi = model.set_variable(var_type='_x', var_name='phi', shape=(1,1)) #heading of car
delta = model.set_variable(var_type='_u', var_name='delta', shape=(1,1))# front steering angle
a = model.set_variable(var_type='_u', var_name='a', shape=(1,1))# accelleration

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
beta=model.set_expression('beta', np.arctan((l_r/l)*np.tan(delta)))

#differential equation
dx_dt= v * np.cos(phi+beta)
dy_dt= v * np.sin(phi+beta)
dphi_dt=(v/l_r)*sin(beta)
dv_dt=a#casadi.SX.zeros(1,1) # a=0
#ddelta_dt=casadi.SX.zeros(1,1)#omega

model.set_rhs('x', dx_dt)
model.set_rhs('y', dy_dt)
model.set_rhs('phi', dphi_dt)
model.set_rhs('v', dv_dt)



#setup
model.setup()
mpc = do_mpc.controller.MPC(model)

#optimiser parameters
setup_mpc = {
    'n_horizon': 20,
    't_step': 0.1,
    'n_robust': 1,
    'store_full_solution': True,
}
mpc.set_param(**setup_mpc)

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

mpc.set_objective(mterm=mterm, lterm=lterm)

mpc.set_rterm(delta=10.)

mpc.set_objective(mterm=mterm, lterm=lterm)

# Constraints on steering angle
mpc.bounds['lower','_u','delta'] = -0.7
mpc.bounds['upper','_u','delta'] = 0.7

mpc.bounds['lower','_u','a'] = -1
mpc.bounds['upper','_u','a'] = 5


mpc.setup()
simulator = do_mpc.simulator.Simulator(model)
simulator.set_param(t_step = 0.1)
simulator.setup()

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

mpc.x0 = state_0
simulator.x0 = state_0



# Set initial guess for MHE/MPC based on initial state.
mpc.set_initial_guess()

mpc_graphics = do_mpc.graphics.Graphics(mpc.data)
sim_graphics = do_mpc.graphics.Graphics(simulator.data)


u0 = np.zeros((2,1))
x0=simulator.x0
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

