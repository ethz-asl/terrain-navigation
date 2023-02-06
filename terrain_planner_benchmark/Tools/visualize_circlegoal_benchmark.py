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

def appendDataset(data_df):
    circle_goal_path_length = getResults(data_df, "circle_goal")/1000.0 # convert m to km
    yaw_goal_path_length = getResults(data_df, "yaw_goal")/1000.0 # convert m to km
    return circle_goal_path_length, yaw_goal_path_length

def boxPlot(ax, idx, data, colors):
    circle_bp = ax.boxplot(data, \
        positions=np.array([idx-0.3, idx+0.3])+ 1.0 * idx + 1.0, \
        widths=0.4, \
        patch_artist=True, \
        notch=0, \
        vert=1, \
        whis=1.5, \
        bootstrap=1000)
    # fill with colors
    for patch, color in zip(circle_bp['boxes'], colors):
        patch.set_facecolor(color)
    ax.set_ylabel('Path Length [km]')
    ax.yaxis.grid(True, which='major', alpha=0.5)
    ticks = ['Sargans', 'Dischma', 'Gotthard']
    ax.set_xticks([1.0, 3.0, 5.0], ticks)
    ax.set_xlim(0, len(ticks)*2)

    return


with open(sys.argv[1]) as file:
    # The FullLoader parameter handles the conversion from YAML
    # scalar values to Python the dictionary format
    list = yaml.load(file, Loader=yaml.FullLoader)

    fig = plt.figure("Coverage", figsize=(5, 3))
    ax = fig.add_subplot(1, 1, 1)
    colors = ['lightblue', 'pink']

    dataset_circle_goal = {}
    dataset_yaw_goal = {}

    idx = 0

    for category_name, category in list.items():
        name = {}
        # map_data_df = pd.read_csv(value['path'])
        for key, value in category.items():
            print("key: ", key, " value: ", value)
            if key == 'name':
                name = value
                continue

            if key == 'path':
                circle_goal, yaw_goal = appendDataset(pd.read_csv(value))
                dataset = [circle_goal, yaw_goal]
                boxPlot(ax, idx, dataset, colors)
                idx = idx + 1
                continue

    custom_lines = [plt.Line2D([0], [0], color=colors[0], lw=4),
                    plt.Line2D([0], [0], color=colors[1], lw=4)]

    ax.legend(custom_lines, ['Circle Goal', 'Yaw Goal'], loc='lower right')
    plt.tight_layout()
    plt.show()
