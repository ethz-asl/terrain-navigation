#!/usr/bin/env python3
# This python script visualizes the geometric priors
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys
import yaml


def visualizeCoverage(ax, name, data_df):

    yaw = np.array(data_df["yaw"])
    yaw_coverage = np.array(data_df["yaw_coverage"])
    circle_coverage = np.array(data_df["circle_coverage"])
    line, = ax.plot(yaw, yaw_coverage, '--', label=name + ' Goal Yaw')
    ax.plot(yaw, circle_coverage, '-', color=line.get_color(), label=name + ' Goal Circle')

with open(sys.argv[1]) as file:
    # The FullLoader parameter handles the conversion from YAML
    # scalar values to Python the dictionary format
    list = yaml.load(file, Loader=yaml.FullLoader)

    dataset_circle_goal = {}
    dataset_yaw_goal = {}

    idx = 0

    fig = plt.figure("Coverage", figsize=(5, 3))
    ax = fig.add_subplot(1, 1, 1)
    for category_name, category in list.items():
        name = {}
        # map_data_df = pd.read_csv(value['path'])
        for key, value in category.items():
            print("key: ", key, " value: ", value)
            if key == 'name':
                name = value
                continue

            if key == 'path':
                data_df = pd.read_csv(value)
                visualizeCoverage(ax, name, data_df)

                continue
    
    ax.set_xlabel('Yaw [rad]')
    ax.set_ylabel('Coverage')
    ax.set_ylim([0.0, 1.0])
    ax.grid(True)
    ax.legend(loc="lower right")


# plt.legend()
plt.tight_layout()
plt.show()
