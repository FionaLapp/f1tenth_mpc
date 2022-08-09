from numpy import linspace
import rosgraph
import os
import time
import csv
import sys

# open the file in the write mode
cwd=sys.path[0]
print(cwd)
csv_filepath=cwd+"/../logs/obs_high_delta"
#header=["node_type","velocity_weight","r_v", "r_delta", "n_horizon", "velocity", "world_name", "include obstacles", "lap1", "lap2","lap3", "collisions", "dnf", "mean_laptime", "std_dev"]

l=[5]
for r in l: #timeout: 5 min (60*5/5)
    print(r)
    if rosgraph.is_master_online(): # Checks the master uri and results boolean (True or False)
        print('ROS MASTER is Online')
    else: #r_delta 4.0
        logfile=csv_filepath+str(r)
        world="Obstacles"+str(r)
        os.system("roslaunch f1tenth_mpc full_simulation.launch world_name:={} node_type:=ftg velocity_weight:=4.5 velocity:=6 r_v:=1.0 r_delta:=5 n_horizon:=5 log_file_name:={}".format(world, logfile))

    
os.system("killall -9 rosmaster")