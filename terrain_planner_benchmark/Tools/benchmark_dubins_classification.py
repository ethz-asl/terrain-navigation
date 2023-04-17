#!/usr/bin/env python3
# This python script visualizes the geometric priors
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
import pandas as pd
import sys
import yaml
from matplotlib.colors import ListedColormap
import seaborn as sns

def adjacent_values(vals, q1, q3):
    upper_adjacent_value = q3 + (q3 - q1) * 1.5
    upper_adjacent_value = np.clip(upper_adjacent_value, q3, vals[-1])

    lower_adjacent_value = q1 - (q3 - q1) * 1.5
    lower_adjacent_value = np.clip(lower_adjacent_value, vals[0], q1)
    return lower_adjacent_value, upper_adjacent_value

data_df = pd.read_csv('/home/jaeyoung/catkin_ws/src/terrain-navigation/output/timing.csv')

classified_time = np.array(data_df["classified"])
unclassified_time = np.array(data_df["exhaustive"])


fig = plt.figure("Coverage", figsize=(5, 2))
ax = fig.add_subplot(1, 1, 1)
dataset = {}
dataset = [unclassified_time, classified_time]

quartile1, medians, quartile3 = np.percentile(dataset, [25, 50, 75], axis=1)
# parts = ax.violinplot(dataset, [2, 4], widths=2, vert=False, showmeans=False, showmedians=False, showextrema=False, bw_method='scott')
sns.violinplot(ax=ax, data=dataset, orient="h", cut=0)


print("Medians", medians)
print("  -  portion", medians[1]/medians[0])
# i=0
# for pc in parts['bodies']:
#     pc.set_color(cmap(norm(i)))
#     pc.set_edgecolor('black')
#     pc.set_alpha(1)
#     i = i+1

# whiskers = np.array([
#     adjacent_values(sorted_array, q1, q3)
#     for sorted_array, q1, q3 in zip(dataset, quartile1, quartile3)])
# whiskers_min, whiskers_max = whiskers[:, 0], whiskers[:, 1]

# inds = [2.0, 4.0]
# ax.scatter(medians, inds, marker='o', color='white', s=3, zorder=3)
# ax.hlines(inds, quartile1, quartile3, color='k', linestyle='-', lw=5)
# ax.hlines(inds, whiskers_min, whiskers_max, color='k', linestyle='-', lw=1)

labels = ['Exhaustive', 'Ours']
ax.yaxis.set_tick_params(direction='out')
ax.yaxis.set_ticks_position('left')
ax.yaxis.set_ticklabels(labels)
# ax.set_yticks([2.0, 4.0])
# ax.set_ylim(0.5, 2.0* len(labels) +1.5)
ax.set_xlim(left=0)
ax.set_xlabel(r'Time [$\mu$s]')
ax.grid(True)
fig.tight_layout(pad=0.5)

plt.show()
