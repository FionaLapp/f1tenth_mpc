from numpy import linspace
import rosgraph
import os
import time
import csv
import sys

# open the file in the write mode
cwd=sys.path[0]
print(cwd)
csv_filepath=cwd+"/../logs/r-delta-tests_ftg"
header=["node_type","r_v", "r_delta", "n_horizon", "velocity", "world_name", "include obstacles", "lap1", "lap2","lap3", "lap4", "lap5", "collisions", "dnf", "mean_laptime", "std_dev"]
with open(csv_filepath, 'w') as f:
    # create the csv writer
    writer = csv.writer(f)

    # write a row to the csv file
    writer.writerow(header)
r_delta_list=linspace(0.5, 2.5, 5)
print(r_delta_list)
for r in r_delta_list: #timeout: 5 min (60*5/5)
    if rosgraph.is_master_online(): # Checks the master uri and results boolean (True or False)
        print('ROS MASTER is Online')
    else:
        os.system("roslaunch f1tenth_mpc full_simulation.launch node_type:=ftg velocity:='7' r_delta:={} log_file_name:={}".format(r, csv_filepath))

    
os.system("killall -9 rosmaster")