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

xupper=20
yupper=0.4

for j in range (6):
    fig=plt.figure(figsize=(xupper,yupper))
    ax=fig.gca()
    plt.xlim(0,xupper)
    plt.ylim(0,yupper)
    lw=7
    #plt.plot([0,xupper], [0,0], linewidth=lw, color="k")

    #plt.plot([0,xupper], [yupper,yupper], linewidth=lw, color="k")
    plt.axis('off')
    for k in range(j):
        for i in range (6):
            circ_rad=yupper/20
            ystart=0.2*(k%2)
            print(ystart)
            x_pos=k*xupper/j+2
            circle = plt.Circle((x_pos, ystart+i*2*(circ_rad+0.001)), circ_rad, color='k')
            ax.add_patch(circle)

    plt.xticks([])
    plt.yticks([])

    cwd=sys.path[0]
    print(cwd)
    filepath=cwd+"/../maps/Obstacles/Obstacles"+str(j)+".png"
    fig.savefig(filepath)