from turtle import width
import numpy as np
import rosgraph
import os
import time
import csv
import sys
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import matplotlib
import matplotlib.colors as colors
import matplotlib.cm as cmx

xupper=100
fig=plt.figure(figsize=(xupper,4))
ax=fig.gca()
plt.xlim(0,xupper)
yupper=4
plt.ylim(0,yupper)
lw=7
#plt.plot([0,xupper], [0,0], linewidth=lw, color="k")

#plt.plot([0,xupper], [yupper,yupper], linewidth=lw, color="k")
plt.axis('off')
for i in range (4):
    circ_rad=0.2
    x_pos=50
    circle = plt.Circle((x_pos, circ_rad+i*2*(circ_rad+0.01)), circ_rad, color='k')
    ax.add_patch(circle)

plt.xticks([])
plt.yticks([])

cwd=sys.path[0]
print(cwd)
filepath=cwd+"/../maps/Obstacles/Obstacles.png"
fig.savefig(filepath)