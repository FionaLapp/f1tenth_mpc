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

import mpc_base_code as mpc_base_code
import helper.visualiser as visualiser
class ControllerWithConstraints(mpc_base_code.BaseController):
    """ This controoller reads in the line data from the centerline file, 
    then callculates wall points for each centerline point using the track-with provided in the custom param file
    """
    def __init__(self, max_speed=None, add_markers=True):
        self.setup_finished=False
        self.add_markers=add_markers
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
            self.index=(distances_to_current_point.argmin()+4) %self.path_length
            self.make_mpc_step(self.state)
            # m=visualiser.GapMarker(self.path_data_x_l[self.index-1:self.index+1], self.path_data_y_l[self.index-1:self.index+1], 1)
            # m.draw_point()
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
        ##
        #self.upper_x=self.model.set_variable(var_type='_tvp', var_name='upper_x', shape=(1,1))
        self.constraint_t_x=self.model.set_variable(var_type='_tvp', var_name='constraint_t_x', shape=(1,1))
        self.constraint_t_y=self.model.set_variable(var_type='_tvp', var_name='constraint_t_y', shape=(1,1))
        
        self.constraint_p_x_lower=self.model.set_variable(var_type='_tvp', var_name='constraint_p_x_lower', shape=(1,1))
        self.constraint_p_x_upper=self.model.set_variable(var_type='_tvp', var_name='constraint_p_x_upper', shape=(1,1))
        self.constraint_p_y_lower=self.model.set_variable(var_type='_tvp', var_name='constraint_p_y_lower', shape=(1,1))
        self.constraint_p_y_upper=self.model.set_variable(var_type='_tvp', var_name='constraint_p_y_upper', shape=(1,1))
        
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
        
        self.upper=self.controller.set_nl_cons('upper', self.constraint_t_x*(self.y-self.constraint_p_y_upper)-self.constraint_t_y*(self.x-self.constraint_p_x_upper), ub=0,soft_constraint=True, penalty_term_cons=1e4)
        self.lower=self.controller.set_nl_cons('lower', -(self.constraint_t_x*(self.y-self.constraint_p_y_lower)-self.constraint_t_y*(self.x-self.constraint_p_x_lower)), ub=0, soft_constraint=True, penalty_term_cons=1e4)
        
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
                else:
                    template["_tvp", k, "constraint_p_y_upper"] = self.path_data_y_r[i]
                    template["_tvp", k, "constraint_p_y_lower"] = self.path_data_y_l[i]
                
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
                elif n_left<n_right:
                    #print("right wall above car")
                    #print("n_left:{}, n_right:{}".format(n_left, n_right))
                    p_x_upper= self.path_data_x_r[i]
                    p_x_lower= self.path_data_x_l[i]
                    p_y_upper= self.path_data_y_r[i]
                    p_y_lower = self.path_data_y_l[i]
                else: #this should never happen because then the lines are identical
                    raise Exception("n_left=n_right")
                template["_tvp", k, "constraint_p_x_upper"] = p_x_upper
                template["_tvp", k, "constraint_p_x_lower"] = p_x_lower
                template["_tvp", k, "constraint_p_y_upper"] = p_y_upper
                template["_tvp", k, "constraint_p_y_lower"] = p_y_lower
            # try:
            #     print(self.controller.nlp_cons)

            # except:
            #     pass
            

        if self.add_markers:
            vis_point=visualiser.TargetMarker(self.path_data_x[self.index], self.path_data_y[self.index], 1)
            vis_point.draw_point()

            #plotting lines:
            factor=5 #completely random length factor for displayed line
            x_line_list=[p_x_upper-factor*self.path_tangent_x[self.index], p_x_upper+factor*self.path_tangent_x[self.index], p_x_lower-factor*self.path_tangent_x[self.index], p_x_lower+factor*self.path_tangent_x[self.index]]
            y_line_list=[p_y_upper-factor*self.path_tangent_y[self.index], p_y_upper+factor*self.path_tangent_y[self.index], p_y_lower-factor*self.path_tangent_y[self.index], p_y_lower+factor*self.path_tangent_y[self.index]]
            constraint_left_marker=visualiser.ConstraintMarker(x_line_list, y_line_list , 1)
            constraint_left_marker.draw_point()

        return template   

    
    def plot_mpc(self, event):
        # self.configure_graphics()
        # self._plotter.plot_results()
        # self._plotter.reset_axes()
        # plt.show()

        # data_array=self.controller.data['_x']
        # print(data_array)
        # x_data=data_array[:,0]
        # y_data=data_array[:,1]

        # fig = plt.figure(figsize=(10,5))

        # plt.plot(x_data, y_data)
        # plt.xlabel('x position')
        # plt.ylabel('y position')
        # title="vehicle_path".format()
        # plt.title(title)
        # plt.show()

        self.configure_graphics()
        self._plotter.plot_results()
        self._plotter.reset_axes()
        plt.show()
        #filepath=self.params['directory']+"/plots/"+title + str(event.current_real)
        #fig.savefig(filepath, bbox_inches='tight', dpi=150)
        
    def configure_graphics(self):
        """
        Matplotlib-based plotter and connect relevant data points to it.
        Additional styling is added for more pleasing visuals and can be extended for custom plotting.
        this function was copied from https://github.com/TheCodeSummoner/f1tenth-racing-algorithms
        """
        rospy.loginfo("Configuring graphics")
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
    
    model_predictive_control =ControllerWithConstraints(max_speed=4, add_markers=False)
    #uncomment beloow to create mpc graph
    #rospy.Timer(rospy.Duration(30), model_predictive_control.plot_mpc)
    rospy.sleep(0.1)
    rospy.spin()
    
    




if __name__=='__main__':
	main(sys.argv)





