import numpy as np
import rosgraph
import os
import time
import csv
import sys
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns



cwd=sys.path[0]
print(cwd)
plot_var="r_delta"
csv_filepath=cwd+"/../logs/"+plot_var+"_tests"

# fig, axs = plt.subplots(1, 1, figsize=(6, 6))

# # Add some labels
fig, ax = plt.subplots(1, 1, figsize=(6, 6))
df=pd.read_csv(csv_filepath)
#label_text="Constants:"
#fig.text(0.5, 0.08, label_text, fontsize=10, ha='center', va='bottom')
# Save figure
#fig.tight_layout()
#fig.subplots_adjust(bottom=0.2)

df_pivot = pd.pivot_table(
	df,
	values="mean_laptime",
	index="r_delta",
	columns="node_type"
)


yerr=pd.pivot_table(
	df,
	values="std_dev",
	index="r_delta",
	columns="node_type"
)
df_pivot.plot(kind="bar", yerr=yerr, ax=ax)
#plt.errorbar(x, y, yerr=None, xerr=None, fmt='', ecolor=None, elinewidth=None, capsize=None, barsabove=False, lolims=False, uplims=False, xlolims=False, xuplims=False, errorevery=1, capthick=None, *, data=None, **kwargs)
# Get a Matplotlib figure from the axes object for formatting purposes

# Change the plot dimensions (width, height)
#fig.set_size_inches(7, 6)
# Change the axes labels
ax.set_xlabel("r_delta")
ax.set_ylabel("Mean Lap time (s)")
plt.title("Mean lap time for varying values of r_delta")
plt.figtext(0.5, -0.3, "Constants:", fontdict=None)
#ax.annotate("constants", ())
plt.ylim(70, 73)

fig.savefig(cwd+"/"+plot_var+".png")