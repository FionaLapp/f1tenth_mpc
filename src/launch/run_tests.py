from numpy import linspace
import rosgraph
import os
import time
import csv
import sys

# open the file in the write mode
cwd=sys.path[0]
print(cwd)
csv_filepath=cwd+"/../logs/n_horizon_cwc2"
header=["node_type","velocity_weight","r_v", "r_delta", "n_horizon", "velocity", "world_name", "include obstacles", "lap1", "lap2","lap3", "collisions", "dnf", "mean_laptime", "std_dev"]
with open(csv_filepath, 'w') as f:
    # create the csv writer
    writer = csv.writer(f)

    # write a row to the csv file
    writer.writerow(header)
param_list=[4]
print(param_list)
for r in param_list: #timeout: 5 min (60*5/5)
    print(r)
    if rosgraph.is_master_online(): # Checks the master uri and results boolean (True or False)
        print('ROS MASTER is Online')
    else: #r_delta 4.0
        os.system("roslaunch f1tenth_mpc full_simulation.launch node_type:=centerline_with_constraints velocity_weight:=1.5 velocity:=7 r_v:=1.0 r_delta:=4.0 n_horizon:={} log_file_name:={}".format(r, csv_filepath))

    
os.system("killall -9 rosmaster")