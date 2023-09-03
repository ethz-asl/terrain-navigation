#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib import cm

# initialize the graph:
file_path = "/home/jaeyoung/catkin_ws/src/terrain-navigation/output/sertig_cross_section.csv"
data_df = pd.read_csv(file_path)

position = data_df["x"].to_numpy()
terrain = data_df["terrain_height"].to_numpy()
min_distance = data_df["minimum_distance"].to_numpy()
max_distance = data_df["maximum_distance"].to_numpy()
circle_lower = data_df["H_-"].to_numpy()
circle_upper = data_df["H_+"].to_numpy()

print(data_df)

fig1 = plt.figure("Terrain and Path Visualization", figsize=(5, 3))
ax1 = fig1.add_subplot(2, 1, 1)
plot_terrain = ax1.plot(position, terrain, label=r"$H$")
plot_min_distance = ax1.plot(position, min_distance, label=r"$D^-$")
plot_max_distance = ax1.plot(position, max_distance, label=r"$D^+$")
# ax1.plot(position, circle_lower)red
# ax1.plot(position, circle_upper)
ax1.set_ylabel('Altitude [m]')
# ax1.axis('equal')
# ax1.set_ylim([1900.0, 2630.0])
# ax1.grid(True)
ax1.set_xlim([np.min(position), np.max(position)])
ax1.legend(ncol=3)
ax1.tick_params(axis='x',          # changes apply to the x-axis
    which='both',      # both major and minor ticks are affected
    bottom=False,      # ticks along the bottom edge are off
    top=False,         # ticks along the top edge are off
    labelbottom=False)

ax2 = fig1.add_subplot(2, 1, 2)
# ax2.plot(position, terrain)
ax2.plot(position, min_distance, '--', color=plot_min_distance[0].get_color(),label=r"$D^-$")
ax2.plot(position, max_distance, '--', color=plot_max_distance[0].get_color(), label=r"$D^+$")
ax2.plot(position, circle_upper, color=plot_min_distance[0].get_color(), label=r"$H^+$")
ax2.plot(position, circle_lower, color=plot_max_distance[0].get_color(), label=r"$H^-$")

ax2.set_ylabel('Altitude [m]')
cmap = cm.get_cmap('winter')

valid_region = []
invalid_region = []
for i in range(1,len(position)):
    if circle_lower[i] >= circle_upper[i]:
        valid_region.append([position[i-1], position[i]])
    elif circle_lower[i] < circle_upper[i]:
        invalid_region.append([position[i-1], position[i]])
for i in valid_region:
    ax2.axvspan(i[0], i[1], -10, 10, facecolor=cmap(1.0), alpha=0.2)
for i in invalid_region:
    ax2.axvspan(i[0], i[1], -10, 10, facecolor=cmap(0.0), alpha=0.2)

# ax2.axis('equal')
# ax2.set_ylim([1900.0, 2630.0])
# ax2.set_ylim([-750, 750])
ax2.set_xlim([np.min(position), np.max(position)])
ax2.set_xlabel('X [m]')

ax2.legend(ncol=2)

plt.tight_layout()

plt.show()

print("\n")
print("Done")