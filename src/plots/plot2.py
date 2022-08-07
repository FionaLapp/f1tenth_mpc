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

fig, ax = plt.subplots(1, 3, figsize=(6, 6))
df=pd.read_csv(csv_filepath)
#df = df.drop(df[df.dnf==True].index)

df.loc[(df.dnf == 'True'),'mean_laptime']= 0
df_FTG= df[df.node_type=='ftg']
plt.bar(df, bins=[1,1,1,1])
plt.show()
ax.set_xlabel(plot_var)
ax.set_ylabel("Mean Lap time (s)")
plt.title("Mean lap time for varying values of "+ plot_var)

plt.ylim(65, 80)
plt.axhline(y=74, xmin=0, xmax= 5/17)
#plt.hlines(87, 0.1, 1.5, color='red')
plt.show()

#fig.savefig(cwd+"/"+plot_var+".png")