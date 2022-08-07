import numpy as np
import rosgraph
import os
import time
import csv
import sys
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns


sns.set_theme(context='notebook', style='darkgrid', palette='deep', font='sans-serif', font_scale=1, color_codes=True, rc=None)


cwd=sys.path[0]
print(cwd)
plot_var="r_delta"
csv_filepath=cwd+"/../logs/"+plot_var+"_tests"
df=pd.read_csv(csv_filepath)

df.mean_laptime = df.mean_laptime.apply(pd.to_numeric, errors='coerce')
df.std_dev = df.std_dev.apply(pd.to_numeric, errors='coerce')
df = df.replace({np.nan: 0})    
fig, ((ax1, ax2, ax3)) = plt.subplots(3, 1, figsize=(9,12), sharey=True)
for ax, node_type, controller in zip([ax1, ax2, ax3], ["ftg", "centerline", "centerline_with_constraints"], ["FTG", "PP1", "PP2"]):
    #df = df.drop(df[df.dnf==True].index)
    df_FTG= df[df.node_type==node_type]
    idx=np.asarray([i for i in range(len(df_FTG[plot_var]))])
    ax.set_xticks(df_FTG[plot_var])
    #print(len(df_FTG[plot_var]))
    ax.set_xticklabels(df_FTG[plot_var], rotation=65)
    #color=(0.5,0.5,0.5,(3-df_FTG['collisions'])*0.3)
    color=[]
    for i in range(len(df_FTG['collisions'])):
        if ax==ax1:
            color.append((1,0,0,(4-df_FTG['collisions'].iloc[i])*0.25))
        if ax==ax2:
            color.append((0,1,0,(4-df_FTG['collisions'].iloc[i])*0.25))
        if ax==ax3:
            color.append((0,0,1,(4-df_FTG['collisions'].iloc[i])*0.25))
        
    #print(df_FTG['mean_laptime'])
    ax.bar(x=df_FTG[plot_var], height=df_FTG['mean_laptime'], width=0.05, yerr=df_FTG['std_dev'], color=color)

    if ax==ax3:
        ax.set_xlabel(plot_var)
    if ax==ax2:
        ax.set_ylabel("mean lap time (s)")
    ax.set_title(controller)
    plt.ylim(70, 73)
    plt.xlim(3,7)

fig.suptitle("Mean lap time for varying values of "+ plot_var)
plt.subplots_adjust(
                    bottom=0.1, 
                    top=0.9, 
                    wspace=0.4, 
                    hspace=0.7)
#plt.axhline(y=74, xmin=0, xmax= 5/17)
#plt.hlines(87, 0.1, 1.5, color='red')
#plt.show()

fig.savefig(cwd+"/"+plot_var+".png")