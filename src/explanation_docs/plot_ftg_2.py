import numpy as np
import matplotlib.pyplot as plt
import csv 
import sys
import pandas as pd

cwd=sys.path[0]
print(cwd)
ranges=[]

csv_filepath=cwd+"/ftg_ranges_example.csv"
#ranges=pd.read_csv(csv_filepath)
with open(csv_filepath, newline='') as csvfile:
    data=csvfile.read()
data_list=data.split(",")
r_array=np.zeros(len(data_list))
for i in range(len(data_list)):
    r_array[i]=float(data_list[i].strip())

x_array=np.linspace(-np.pi, np.pi, num=len(r_array), endpoint=True, retstep=False, dtype=None, axis=0)
r_array=np.where(r_array<5, r_array, 0*r_array)
fig=plt.figure(figsize=(10,6))
plt.xticks(fontsize=16)
plt.yticks(fontsize=16)
plt.xlabel("lidar angle (rad)", fontsize=18)
plt.ylabel("lidar range (m)", fontsize=18)
plt.title("Plot of lidar range data vs angle", fontsize=20)
plt.plot(x_array, r_array)
#plt.show()
plt.savefig(cwd+"/ftg_plot.png")
print("done")