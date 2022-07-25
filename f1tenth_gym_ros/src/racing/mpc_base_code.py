#!/usr/bin/env python3
"""
Created on Thu May 12 12:06:18 2022
@author: Fiona
"""
from __future__ import print_function
import sys
import numpy as np
from abc import ABC, abstractmethod

#MPC imports
# Add do_mpc to path. This is not necessary if it was installed via pip.
import sys
sys.path.append('../../')
import os.path

import matplotlib.pyplot as plt
import casadi

# Import do_mpc package:
from do_mpc.model import Model
from do_mpc.controller import MPC
from do_mpc.data import MPCData

from do_mpc.graphics import Graphics
import matplotlib.pyplot as plt
import matplotlib


#ROS Imports
import rospy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

  
# importing 
import helper.visualiser as visualiser

class BaseController(ABC):
    """ 
    """
    def __init__(self, max_speed=None, add_markers=True, time_step=0.1):
        self.setup_finished=False
        self.current_steering_angle=0
        self.add_markers=add_markers
        self.params=self.get_params()
        self.setup_node()
        if max_speed is None:
            max_speed=self.params['max_speed']
        self.setup_mpc(max_speed=max_speed, n_horizon=self.params['n_horizon'], time_step=time_step)
        self.setup_finished=True
        
       


    def setup_node(self):
        #Topics & Subs, Pubs
        
        localisation_topic= 'ego_racecar/odom' #change to a different topic if applicable (e.g. if using hector)
        drive_topic = '/drive'


        self.localisation_sub=rospy.Subscriber(localisation_topic, Odometry, self.localisation_callback)
        self.fixing_a_weird_bug_and_not_much_else_sub=rospy.Subscriber('/odom', Odometry, self.pose_callback, queue_size=1)#subscribing to /odom a second time somehow makes the first one work, otherwise it gets stuck at the origin
    
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        
        
        
    
    def get_params(self):
        #I quite possibly came up with the worst way to do this
        rospy.loginfo("Getting params")
        params={}
        print("Ns:{}".format(rospy.get_namespace()))
        namespace=rospy.get_namespace()+'mpc/'
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
        
        #custom stuff
        params['directory']=rospy.get_param(namespace+'directory')
        params['ftg_safety_raduis']=rospy.get_param(namespace+'ftg_safety_raduis')
        params['center_to_wall_distance']=rospy.get_param(namespace+'center_to_wall_distance')
        params['n_horizon']=rospy.get_param(namespace+'n_horizon', 5)
        return params    

    def make_mpc_step(self, x_state):
        if not self.setup_finished or not self.controller.flags['setup']:
            rospy.logdebug("setup not finished, can't make step")
            return

        #making the mpc calculation
        u =self.controller.make_step(x_state) 

        #plotting the predicted  trajectorry
        x_pred=self.controller.data.prediction(('_x', 'x')).flatten()
        y_pred=self.controller.data.prediction(('_x', 'y')).flatten()
        if self.add_markers:
            vis_point=visualiser.TrajectoryMarker(x_pred, y_pred, 1)  #somehow this dooesn't show up in the right colour or line thickness but for now it'll do
            vis_point.draw_point()
            
        #sending control input to /drive topic
        delta=u[0]
        self.current_steering_angle=delta
        v=u[1]
        rospy.loginfo("{}, {}".format(u[0], u[1]))
        #setup drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "drive"
        drive_msg.drive.steering_angle = delta
        drive_msg.drive.speed=v
        self.drive_pub.publish(drive_msg)



    def pose_callback(self,pose_msg):
        a=pose_msg  
        #print(pose_msg) 

        
    @abstractmethod
    def localisation_callback(self, data:Odometry):
        """
        Could be from any source of localisation (e.g. odometry or lidar)---> adapt get_state_from_data metod accordingly
        Please call make_mpc_step in this method and update goal
        """
        pass
        

    def setup_mpc(self, max_speed, n_horizon, time_step):
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
        self.target_x=self.model.set_variable(var_type='_tvp', var_name='target_x', shape=(1,1))
        self.target_y=self.model.set_variable(var_type='_tvp', var_name='target_y', shape=(1,1))
        
        self.wall_distance=self.model.set_variable(var_type='_tvp', var_name='wall_distance', shape=(1,1))
        
        self.measured_steering_angle=self.model.set_variable(var_type='_tvp', var_name='measured_steering_angle', shape=(1,1))
        self.curvature=self.model.set_variable(var_type='_tvp', var_name='curvature', shape=(1,1))
        
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
            't_step': time_step,
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
        target_x_list=[]
        target_y_list=[]
        for k in range(self.n_horizon + 1):
            i=(self.index+k)%self.path_length
            template["_tvp", k, "target_x"]=self.path_data[' x_m'][i]
            template["_tvp", k, "target_y"] =self.path_data[' y_m'][i]
            target_x_list.append(self.path_data_x[i])
            target_y_list.append(self.path_data_y[i])
            
            #template["_tvp", k, "wall_distance"] =self.distance_to_wall_ahead
            
            template["_tvp", k, "measured_steering_angle"] =self.current_steering_angle
            try:
                template["_tvp", k, "curvature"] =self.curvature_array[i]
            except Exception:
                pass #ftg doesn't have curvature array
            #print("phi:{}".format(self.state[2]))
        if self.add_markers:
            vis_point=visualiser.TargetMarker(target_x_list, target_y_list, 1)
            
            #vis_point=visualiser.TargetMarker(self.path_data_x[self.index:self.index+self.n_horizon], self.path_data_y[self.index:self.index+self.n_horizon], 1)
            vis_point.draw_point()
        #rospy.loginfo("template prepared with goal (x,y)= ({}, {})".format(self.goal_x, self.goal_y))    
        
        return template    

        

   

    @property
    def stage_cost(self):
        """
        none
        """
        #return (self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2 +13*self.measured_steering_angle*self.v #+(200/self.wall_distance)*self.v
        return (self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2 +(4/self.wall_distance)*self.v**2 #+(200/self.wall_distance)*self.v
        
    @property
    def terminal_cost(self):
        """
        difference between target and actual
        """
        
        return (self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2 
        




    def plot_mpc(self, event):
        

        # data_array=self.controller.data['_x']
        # print(data_array)
        # x_data=data_array[:,0]
        # y_data=data_array[:,1]

        #ig = plt.figure(figsize=(10,5))

        # plt.plot(x_data, y_data)
        # plt.xlabel('x position')
        # plt.ylabel('y position')
        # title="vehicle_path".format()
        # plt.title(title)
        # plt.show()

        fig=self.configure_graphics()
        self._plotter.plot_results()
        self._plotter.reset_axes()
        plt.show()
        title="parameter_plot"
        directory=self.params['directory']+ "/src/figures/"
        filepath=os.path.expanduser(directory+title + str(event.current_real)+".png")
        
        #filepath=os.path.join(self.params['directory']), ("src"), ("figures"), (title + str(event.current_real))
        #filepath="/figures/"+title + str(event.current_real)
        
        print(filepath)
        fig.savefig(filepath, bbox_inches='tight', dpi=150)
        plt.show()
        
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
        #self._plotter.add_line(var_type="_x", var_name="phi", axis=axis[1], color="red")
        self._plotter.add_line(var_type="_u", var_name="delta", axis=axis[1], color="green")
        self._plotter.add_line(var_type="_u", var_name="v", axis=axis[2], color="red")

        # Set X and Y labels
        axis[0].set_ylabel("Position")
        axis[1].set_ylabel("Angles")
        axis[2].set_ylabel("Velocity")
        axis[2].set_xlabel("Time")
        return figure