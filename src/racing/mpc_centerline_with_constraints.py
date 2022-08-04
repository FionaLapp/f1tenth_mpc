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



#MPC imports
# Add do_mpc to path. This is not necessary if it was installed via pip.
import sys
sys.path.append('../../')

import casadi

# Import do_mpc package:
from do_mpc.model import Model
from do_mpc.controller import MPC

#ROS Imports
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from rospy.rostime import Duration, Time

import mpc_base_code as mpc_base_code
import helper.visualiser as visualiser
class ControllerWithConstraints(mpc_base_code.BaseController):
    """ This controoller reads in the line data from the centerline file, 
    then callculates wall points for each centerline point using the track-with provided in the custom param file
    """
    def __init__(self, add_markers=True, time_step=0.1):
        self.setup_finished=False
        self.add_markers=add_markers
        self.laps_completed=0
        self.params=super().get_params()
        self.read_desired_path()
        self.calculate_wall_points()
        self.setup_node()
        if self.params['velocity']<= self.params['max_speed']:
            max_speed=self.params['velocity']
        else:
            rospy.loginfo("Can't go that fast, only able to drive {}m/s but you requested {}m/s. I'll drive as fast as I can though :-)".format(self.params['max_speed'], self.params['velocity']))
            max_speed=self.params['max_speed']        
        self.state=[0,0,0]
        self.setup_mpc(max_speed=max_speed, time_step=time_step, n_horizon=self.params['n_horizon'])
        

        self.key_pub.publish(String("n"))
        self.lap_start_time=Time.now()
        
        self.setup_finished=True


 

    def  calculate_wall_points(self):
        """ take the corresponding centerline point, roll it over my one, and subtract to get tangent vectors, create a vector perpendicular to it by switching 
        coordinates and either sign (so the dot prodict is 0), divide by length to get unit vector, multiply 
        by half the track width. add that vector to the original centerline point.
        """
        #first derivatives 
        self.path_tangent_x= np.gradient(self.path_data_x)#self.path_data_x-np.roll(self.path_data_x, 1)
        self.path_tangent_y= np.gradient(self.path_data_y)#self.path_data_y-np.roll(self.path_data_y, 1)
        
        #track borders
        l=np.sqrt(self.path_tangent_x**2+self.path_tangent_y**2)
        self.path_data_x_r=self.path_data_x+self.path_tangent_y*self.trackwidth/l
        self.path_data_y_r=self.path_data_y-self.path_tangent_x*self.trackwidth/l
        self.path_data_x_l=self.path_data_x-self.path_tangent_y*self.trackwidth/l
        self.path_data_y_l=self.path_data_y+self.path_tangent_x*self.trackwidth/l
        
        #first derivatives
        dx=self.path_tangent_x
        dy=self.path_tangent_y

        #second derivatives 
        d2x = np.gradient(dx)
        d2y = np.gradient(dy)

        #calculation of curvature 
        curvature = np.abs(dx * d2y - d2x * dy) / (dx * dx + dy * dy)**1.5
        



    def calculate_curvature(self):
        pass

    def localisation_callback(self, data:Odometry):
        """
        Could be from any source of localisation (e.g. odometry or lidar)---> adapt get_state_from_data metod accordingly
        """
        try:#update current state
            self.double_previous_delta=self.previous_delta
            self.previous_x=self.state[0]
            self.previous_y=self.state[1]
            self.previous_delta=self.state[2]
            x=data.pose.pose.position.x
            y=data.pose.pose.position.y
            orientation_list=[data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
            (roll, pitch, phi) = euler_from_quaternion (orientation_list)
            #rospy.loginfo("{}, {}, {}".format(x, y, phi))
            self.state= np.array([x,y, phi])
            
            #update target: find the point on the centerline fiile closest to the current position, then go two further
            distances_to_current_point=(self.path_data_x-self.state[0])**2+(self.path_data_y-self.state[1])**2
            closest=(distances_to_current_point.argmin()+1) #not actually the closest because we want to always be ahead
            self.index= closest+1 %self.path_length
            if np.abs(closest - self.path_length)<10:
                super().on_lap_complete()
            self.make_mpc_step(self.state)
            # m=visualiser.GapMarker(self.path_data_x_l[self.index-1:self.index+1], self.path_data_y_l[self.index-1:self.index+1], 1)
            # m.draw_point()
        except AttributeError:
            rospy.loginfo("Initialisation not finished")


    def setup_mpc(self, max_speed, time_step, n_horizon):
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
        self.curvature=self.model.set_variable(var_type='_tvp', var_name='curvature', shape=(1,1))
        

        self.previous_delta=self.model.set_variable(var_type='_tvp', var_name='previous_delta', shape=(1,1))
        
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
        
        self.upper=self.controller.set_nl_cons('upper', -(self.constraint_t_x*(self.y-self.constraint_p_y_upper)-self.constraint_t_y*(self.x-self.constraint_p_x_upper)), ub=0,soft_constraint=True, penalty_term_cons=1e4)
        self.lower=self.controller.set_nl_cons('lower', (self.constraint_t_x*(self.y-self.constraint_p_y_lower)-self.constraint_t_y*(self.x-self.constraint_p_x_lower)), ub=0, soft_constraint=True, penalty_term_cons=1e4)
        
        self.controller.set_objective(lterm=self.stage_cost, mterm=self.terminal_cost)
        self.controller.set_rterm(v=self.params['r_v'])
        self.controller.set_rterm(delta=self.params['r_delta'])

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
            template["_tvp", k, "target_x"]=self.path_data_x[i]
            template["_tvp", k, "target_y"] =self.path_data_y[i]
            try:
                #print(self.curvature_array[i%self.path_length])
                template["_tvp", k, "curvature"] =self.curvature_array[i%self.path_length]
            except Exception:
                pass
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
            j=i#(i-k)%self.path_length
            template["_tvp", k, "constraint_t_x"] = self.path_tangent_x[j]
            template["_tvp", k, "constraint_t_y"] = self.path_tangent_y[j]
            #print(self.path_tangent_x[i])
            if abs(self.path_tangent_x[j])==0:
                print("x tangent=0")

                if self.path_tangent_y[j]==0:
                    raise Exception("Tangent vector to wall is 0 vector")
                y_l=self.path_data_y_l[j]
                y_r=self.path_data_y_r[j]
                if y_l>y_r:
                    p_y_lower=self.path_data_y_r[j]
                    p_x_upper=self.path_data_y_l[j]
                    
                    p_x_lower=self.x
                    p_x_upper=self.x
                    template["_tvp", k, "constraint_p_x_lower"] = self.x
                    template["_tvp", k, "constraint_p_x_upper"] = self.x
                
                else:
                    p_y_upper= self.path_data_y_r[j]
                    p_y_lower= self.path_data_y_l[j]
                    template["_tvp", k, "constraint_p_x_lower"] = self.x
                    template["_tvp", k, "constraint_p_x_upper"] = self.x
                    p_x_lower=self.x
                    p_x_upper=self.x
                template["_tvp", k, "constraint_p_y_upper"] = p_y_upper
                template["_tvp", k, "constraint_p_y_lower"] = p_y_lower
            else:
                
                #calculate y-value of x-value corresponding to current car position. if current car y < line y --> point below line
                n_left=(self.path_data_y_l[i]-(self.path_tangent_y[j]/self.path_tangent_x[j])*self.path_data_x_l[j])
                n_right=(self.path_data_y_r[i]-(self.path_tangent_y[j]/self.path_tangent_x[j])*self.path_data_x_r[j])
                if n_left>n_right:
                    #print("n_left:{}, n_right:{}".format(n_left, n_right))
                    #print("left wall above car")
                    p_x_upper= self.path_data_x_l[j]
                    p_x_lower = self.path_data_x_r[j]
                    p_y_upper = self.path_data_y_l[j]
                    p_y_lower = self.path_data_y_r[j]
                elif n_left<n_right:
                    #print("right wall above car")
                    #print("n_left:{}, n_right:{}".format(n_left, n_right))
                    p_x_upper= self.path_data_x_r[j]
                    p_x_lower= self.path_data_x_l[j]
                    p_y_upper= self.path_data_y_r[j]
                    p_y_lower = self.path_data_y_l[j]
                else: #this should never happen because then the lines are identical
                    raise Exception("n_left=n_right")
                template["_tvp", k, "constraint_p_x_upper"] = p_x_upper
                template["_tvp", k, "constraint_p_x_lower"] = p_x_lower
                template["_tvp", k, "constraint_p_y_upper"] = p_y_upper
                template["_tvp", k, "constraint_p_y_lower"] = p_y_lower

                template["_tvp", k, "previous_delta"] =self.previous_delta
               
            # try:
            #     print(self.controller.nlp_cons)

            # except:
            #     pass
            

            if self.add_markers:
                
                target_marker_list_x.append(self.path_data_x[i])
                
                target_marker_list_y.append( self.path_data_y[i])
                #plotting lines:
                factor=5 #completely random length factor for displayed line
                #x_line_list.extend([p_x_upper-factor*self.path_tangent_x[j], p_x_upper+factor*self.path_tangent_x[j]])
                x_line_list.extend( [p_x_lower-factor*self.path_tangent_x[i], p_x_lower+factor*self.path_tangent_x[i]])
                #y_line_list.extend([p_y_upper-factor*self.path_tangent_y[j], p_y_upper+factor*self.path_tangent_y[j]])
                y_line_list.extend( [p_y_lower-factor*self.path_tangent_y[i], p_y_lower+factor*self.path_tangent_y[i]])
        
        if self.add_markers:
            vis_point=visualiser.TargetMarker(target_marker_list_x, target_marker_list_y, 1)
            vis_point.draw_point()
            constraint_left_marker=visualiser.ConstraintMarker(x_line_list, y_line_list , 1)
            constraint_left_marker.draw_point()

        return template   

    
    @property
    def stage_cost(self):
        """
        none
        """
        #return (self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2 +13*self.measured_steering_angle*self.v #+(200/self.wall_distance)*self.v
        return (self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2 +self.params['velocity_weight']*(self.curvature)*self.v#(4/self.wall_distance)*self.v**2 #+(200/self.wall_distance)*self.v
      

  
def main(args):
    
    
    rospy.init_node("mpc_node", anonymous=True)
    rospy.loginfo("starting up mpc node")
    time_step=0.1
    model_predictive_control =ControllerWithConstraints(add_markers=True, time_step=time_step)
    #uncomment below to create mpc graph
    #rospy.Timer(rospy.Duration(10), model_predictive_control.plot_mpc)
    rospy.sleep(time_step)
    rospy.spin()
    
    




if __name__=='__main__':
	main(sys.argv)





