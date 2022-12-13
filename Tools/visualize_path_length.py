#!/usr/bin/env python3
# This python script visualizes the geometric priors
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys
import yaml

def getResults(data_df, method):
    path_length = np.array(data_df["path_length"])
    method_mask = np.array(data_df["planning_method"]) == method
    nan_mask = ~np.isnan(path_length)
    masked_path_length = path_length[method_mask & nan_mask]
    return masked_path_length


data_df = pd.read_csv('/home/jaeyoung/catkin_ws/src/terrain-navigation/output/output.csv')

circle_goal_path_length = getResults(data_df, "circle_goal")
print("circle goal mean: ", circle_goal_path_length.mean(), "+- ", circle_goal_path_length.std())
yaw_goal_path_length = getResults(data_df, "yaw_goal")
print("yaw_goal_path_length: ", yaw_goal_path_length.mean(), "+- ", yaw_goal_path_length.std())


fig = plt.figure("Coverage")
ax = fig.add_subplot(1, 1, 1)
dataset = {}
dataset = [circle_goal_path_length, yaw_goal_path_length]
vp = ax.violinplot(dataset, [2, 4], widths=2)

# labels = ['A', 'B', 'C', 'D']
# ax.xaxis.set_tick_params(direction='out')
# ax.xaxis.set_ticks_position('bottom')
# ax.set_xticks(np.arange(1, len(labels) + 1))
# ax.set_xlim(0.5, len(labels) + 0.5)
# ax.set_xlabel('Sample name')

plt.show()
