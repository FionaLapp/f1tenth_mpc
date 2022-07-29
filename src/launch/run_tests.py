import rosgraph
import os
import time

os.system("roslaunch f1tenth_mpc full_simulation.launch node_type:=centerline velocity:='8'")
for i in range(1000):
    if rosgraph.is_master_online(): # Checks the master uri and results boolean (True or False)
        print('ROS MASTER is Online')
    else:
        print ('ROS MASTER is Offline')
    time.sleep(5)