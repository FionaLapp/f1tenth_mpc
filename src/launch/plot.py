import numpy as np
import rosgraph
import os
import time
import csv
import sys
import pandas as pd
import matplotlib.pyplot as plt



cwd=sys.path[0]
print(cwd)
csv_filepath=cwd+"/../logs/r_delta_tests_centerline"

data=pd.read_csv(csv_filepath)
x=data.r_delta
y=data.mean_laptime

plt.scatter(x,y)
plt.show()