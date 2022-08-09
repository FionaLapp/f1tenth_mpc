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


sns.set_theme(context='notebook', style='darkgrid', palette='deep', font='sans-serif', font_scale=1, color_codes=True, rc=None)


cwd=sys.path[0]
print(cwd)
num_obs=1
csv_filepath=cwd+"/../logs/obs"+str(num_obs)#+plot_var+"curve"
df=pd.read_csv(csv_filepath)

#df.mean_laptime = df.mean_laptime.apply(pd.to_numeric, errors='coerce')
#df.std_dev = df.std_dev.apply(pd.to_numeric, errors='coerce')
#df = df.replace({np.nan: 0})    
fig, ((ax1, ax2, ax3)) = plt.subplots(3, 1, figsize=(8, 5))
cmap = plt.get_cmap('rainbow')
cNorm  = colors.Normalize(vmin=3, vmax=7)
scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=cmap)
ax1.set_clip_on(False)

ax1.annotate("y (m)", (-4, 1), fontsize="medium", annotation_clip=False)
ax1.annotate("v (m/s)", ( df.x.max()+4, 1), fontsize="medium", annotation_clip=False)

#ax1.plot(-df.x_c[9:], df.y_c[9:], linewidth=12, color=(0,0,0,0.25), label="track", zorder=-1)#ftg
#ax1.plot(-df.x_c[165:], df.y_c[165:], linewidth=12, color=(0,0,0,0.25), label="track", zorder=-1)#pp1
#ax1.plot(-df.x_c[50:], df.y_c[50:], linewidth=12, color=(0,0,0,0.25), label="track", zorder=-1)#pp2
j=num_obs
xupper=20
yupper=0.4
factor=10
for k in range(j):
        for i in range (6):
            circ_rad=yupper/20
            ystart=0.2*(k%2)
            print(ystart)
            x_pos=k*xupper/j+2
            circle = plt.Circle((factor*x_pos, 0.5*factor*(ystart+i*2*(circ_rad+0.001))), factor*circ_rad, color='k')
            ax1.add_patch(circle)


ax1.scatter(df.x, df.y, s=3, c=scalarMap.to_rgba(df.v), cmap='rainbow', label="path taken by FTG", norm=matplotlib.colors.SymLogNorm(linthresh=1, vmin=0, vmax=7))
# ax1.arrow(50, -50, 10, 0, color="k", label="direction of travel", length_includes_head=True,
#           head_width=2.5, head_length=2.5)
# #ax1.annotate("direction of travel", xy=(80, -50), xytext=(50, -55),arrowprops=dict(arrowstyle="->"), color="k")
scalarMap.set_array([])
fig.colorbar(scalarMap,ax=ax1)
ax1.legend()

t=np.arange(len(df.v))*0.1
ax1.set_ylim(0,2)
ax2.plot(t, df.v)
ax3.plot(t, df.delta)
ax1.set_xlabel("x (m)")
ax2.set_ylabel("y (m)")
ax1.set_title("Controller path")
ax2.set_xlabel("time (s)")
ax3.set_xlabel("time (s)")
ax2.set_ylabel("speed (m/s)")
ax3.set_ylabel("steering angle (rad)")
ax2.set_title("speed vs time")
ax3.set_title("steering angle vs time")
# for ax, node_type, controller in zip([ax1, ax2, ax3], ["ftg", "centerline", "centerline_with_constraints"], ["FTG", "PP1", "PP2"]):
#     #df = df.drop(df[df.dnf==True].index)
#     df_FTG= df[df.node_type==node_type]
#     idx=np.asarray([i for i in range(len(df_FTG[plot_var]))])
#     ax.set_xticks([3,4,5,6,7])
#     #print(len(df_FTG[plot_var]))
#     ax.set_xticklabels(df_FTG[plot_var])
#     #color=(0.5,0.5,0.5,(3-df_FTG['collisions'])*0.3)
#     color=[]
#     for i in range(len(df_FTG['collisions'])):
#         if ax==ax1:
#             color.append((1,0,0,(4-df_FTG['collisions'].iloc[i])*0.25))
#         if ax==ax2:
#             color.append((0,1,0,(4-df_FTG['collisions'].iloc[i])*0.25))
#         if ax==ax3:
#             color.append((0,0,1,(4-df_FTG['collisions'].iloc[i])*0.25))
        
#     #print(df_FTG['mean_laptime'])
#     ax.bar(x=df_FTG[plot_var], height=df_FTG['mean_laptime'], yerr=df_FTG['std_dev'], color=color)

#     if ax==ax2:
#         ax.set_xlabel(plot_var)
#     if ax==ax1:
#         ax.set_ylabel("mean lap time (s)")
#     ax.set_title(controller)
#     plt.ylim(67, 85)
#     plt.xlim(3,7)

fig.suptitle("FTG obstacle test with "+ str(num_obs)+" obstacles")
plt.subplots_adjust(
                    bottom=0.1, 
                    top=0.85, 
                    wspace=0.0, 
                    hspace=1.3)
#plt.axhline(y=74, xmin=0, xmax= 5/17)
#plt.hlines(87, 0.1, 1.5, color='red')
plt.show()

#fig.savefig(cwd+"/obs_"+str(num_obs)+".png")