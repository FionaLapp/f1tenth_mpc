#!/usr/bin/env python3
"""
Created on Thu May 12 12:06:18 2022
@author: Fiona
"""
from __future__ import print_function
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

import mpc_base_code as mpc_base_code
import helper.visualiser as visualiser
class ControllerWithConstraints(mpc_base_code.BaseController):
    """ This controoller reads in the line data from the centerline file, 
    then callculates wall points for each centerline point using the track-with provided in the custom param file
    """
    def __init__(self, max_speed=None):
        self.setup_finished=False
        self.params=super().get_params()
        self.read_desired_path()
        super().setup_node()
        if max_speed is None:
            max_speed=self.params['max_speed']
        self.state=[0,0,0]
        self.setup_mpc(max_speed=max_speed)
        
        self.setup_finished=True
        
    def read_desired_path(self):
        pathfile_name=rospy.get_param('/mpc/directory')+'/src/maps/Sochi/Sochi_centerline.csv'
        self.path_data=pd.read_csv(pathfile_name)
        self.path_length=self.path_data.shape[0]
        self.path_data_x=self.path_data[' x_m'].to_numpy()
        self.path_data_y=self.path_data[' y_m'].to_numpy()                                            
        self.previous_x=0
        self.previous_y=0
        self.distance_travelled=0.0
        self.index=0
        self.trackwidth=self.params['center_to_wall_distance']-0.1 # thee 0.1 is a random number I decided on for safety
        self.calculate_wall_points()

    def  calculate_wall_points(self):
        """ take the corresponding centerline point, roll it over my one, and subtract to get tangent vectors, create a vector perpendicular to it by switching 
        coordinates and either sign (so the dot prodict is 0), divide by length to get unit vector, multiply 
        by half the track width. add that vector to the original centerline point.
        """
        self.path_tangent_x= self.path_data_x-np.roll(self.path_data_x, 1)
        self.path_tangent_y= self.path_data_y-np.roll(self.path_data_y, 1)
        l=np.sqrt(self.path_tangent_x**2+self.path_tangent_y**2)
        self.path_data_x_r=self.path_data_x+self.path_tangent_y*self.trackwidth/l
        self.path_data_y_r=self.path_data_y-self.path_tangent_x*self.trackwidth/l
        self.path_data_x_l=self.path_data_x-self.path_tangent_y*self.trackwidth/l
        self.path_data_y_l=self.path_data_y+self.path_tangent_x*self.trackwidth/l
        


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
            
            #update target: find the point on the centerline fiile closest to the current position, then go two further
            distances_to_current_point=(self.path_data_x-self.state[0])**2+(self.path_data_y-self.state[1])**2
            self.index=distances_to_current_point.argmin()+2 %len(self.path_data_x)
            self.make_mpc_step(self.state)
            m=visualiser.GapMarker(self.path_data_x_l[self.index-1:self.index+1], self.path_data_y_l[self.index-1:self.index+1], 1)
            m.draw_point()
        except AttributeError:
            rospy.loginfo("Initialisation not finished")


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
        self.upper_x=self.model.set_variable(var_type='_tvp', var_name='upper_x', shape=(1,1))
        self.upper_y=self.model.set_variable(var_type='_tvp', var_name='upper_y', shape=(1,1))
        self.lower_x=self.model.set_variable(var_type='_tvp', var_name='lower_x', shape=(1,1))
        self.lower_y=self.model.set_variable(var_type='_tvp', var_name='lower_y', shape=(1,1))

        self.tvp_path_data_y_l=self.model.set_variable(var_type="_tvp", var_name="tvp_path_data_y_l", shape=(1,1))
        self.tvp_path_data_x_l=self.model.set_variable(var_type="_tvp", var_name="tvp_path_data_x_l", shape=(1,1))
        self.tvp_path_tangent_y=self.model.set_variable(var_type="_tvp", var_name="tvp_path_tangent_y", shape=(1,1))
        self.tvp_path_tangent_x=self.model.set_variable(var_type="_tvp", var_name="tvp_path_tangent_x", shape=(1,1))
        
            

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
        
        #self.controller.bounds['lower','_x','y'] = self.lower_y
        
        #self.controller.set_nl_cons('upper', -self.upper_y, ub = 0)
        self.controller.set_nl_cons('upper', self.tvp_path_data_y_l+(self.tvp_path_tangent_y/self.tvp_path_tangent_x)*(self.state[0]-self.tvp_path_data_x_l), ub = 0)
        self.controller.set_objective(lterm=self.stage_cost, mterm=self.terminal_cost)
        self.controller.set_rterm(v=1)
        self.controller.set_rterm(delta=1)

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
        
        for k in range(self.n_horizon + 1):
            i=(self.index)%self.path_length
            template["_tvp", k, "target_x"]=self.path_data[' x_m'][i]
            template["_tvp", k, "target_y"] =self.path_data[' y_m'][i]
            #equation of line
            #r=p+lambda*t (r: any point on line, p: knownn point on line, lambda: param, t: tangent to line)
            #i.e.
            #r_x=p_x+lambda*t_x
            #r_y=p_y+lambda*t_y
            #eliminate lambda
            #lambda=(r_x-p_x)/t_x
            #r_y=p_y+t_y*(r_x-p_x)/t_x
            template["_tvp", k, "tvp_path_data_y_l"] =self.path_data_y_l[i]
            template["_tvp", k, "tvp_path_data_x_l"] =self.path_data_x_l[i]
            template["_tvp", k, "tvp_path_tangent_x"] =self.path_tangent_x[i]
            template["_tvp", k, "tvp_path_tangent_y"] =self.path_tangent_y[i]
            
            #template["_tvp", k, "upper_y"] =(self.path_data_y_l[i]+(self.path_tangent_y[i]/self.path_tangent_x[i])*(self.x-self.path_data_x_l[i]))
            #template["_tvp", k, "lower_y"] =self.lower_y[i]+(self.path_tangent_y[i]/self.path_tangent_x[i])*(self.x-self.lower_x[i])
            
            #constraints x
            # if self.path_data_x_l[self.index]>self.path_data_x_r[self.index]:
            #     template["_tvp", k, "upper_x"] =self.path_data_x_l[(self.index)%self.path_length]
            #     template["_tvp", k, "lower_x"] =self.path_data_x_r[(self.index)%self.path_length]
            # else:
            #     template["_tvp", k, "upper_x"] =self.path_data_x_r[(self.index)%self.path_length]
            #     template["_tvp", k, "lower_x"] =self.path_data_x_l[(self.index)%self.path_length]
            # #constraints y
            # if self.path_data_y_l[self.index]>self.path_data_y_r[self.index]:
            #     template["_tvp", k, "upper_y"] =self.path_data_y_l[(self.index)%self.path_length]
            #     template["_tvp", k, "lower_y"] =self.path_data_y_r[(self.index)%self.path_length]
            # else:
            #     template["_tvp", k, "upper_y"] =self.path_data_y_r[(self.index)%self.path_length]
            #     template["_tvp", k, "lower_y"] =self.path_data_y_l[(self.index)%self.path_length]
            
        vis_point=visualiser.TargetMarker(self.path_data_x[(self.index+self.n_horizon)%self.path_length], self.path_data_y[(self.index+self.n_horizon)%self.path_length], 1)
        #TODO just about everything, I don't think this works yet
        m=visualiser.GapMarker(self.path_data_x_l[self.index-1:(self.index+1)%len(self.path_data_x)], self.path_data_y_l[self.index-1:(self.index+1)%len(self.path_data_x)], 1)
        m.draw_point()

        #vis_point=visualiser.TargetMarker(self.path_data_x[self.index:self.index+self.n_horizon], self.path_data_y[self.index:self.index+self.n_horizon], 1)
        vis_point.draw_point()
        #rospy.loginfo("template prepared with goal (x,y)= ({}, {})".format(self.goal_x, self.goal_y))    
        return template   
    def plot_mpc(self):
        self.configure_graphics()
        self._plotter.plot_results()
        self._plotter.reset_axes()
        plt.show()

        data_array=self.controller.data['_x']
        x_data=data_array[:,0]
        y_data=data_array[:,1]

        fig = plt.figure(figsize=(10,5))
        plt.plot(x_data, y_data)
        plt.xlabel('x position')
        plt.ylabel('y position')
        title="vehicle path".format()
        plt.title(title)
        plt.show()
        filepath=self.params['directory']+"/plots/"+title
        fig.savefig(filepath, bbox_inches='tight', dpi=150)
        
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

  
def main(args):
    
    
    rospy.init_node("mpc_node", anonymous=True)
    rospy.loginfo("starting up mpc node")
    model_predictive_control =ControllerWithConstraints()
    rospy.on_shutdown(model_predictive_control.plot_mpc())
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)





