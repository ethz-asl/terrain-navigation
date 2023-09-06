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
    line, = ax.plot(yaw, yaw_coverage, '--', label=name + ' Position+Yaw')
    ax.plot(yaw, circle_coverage, '-', color=line.get_color(), label=name + ' Circle Set')

    return circle_coverage, yaw_coverage

def boxPlot(ax, idx, data, colors):
    circle_bp = ax.boxplot(data, \
        positions=np.array([idx+0.3])+ 1.0 * idx + 1.0, \
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
    print("idx: ", idx)
    print("  - Position + heading: ")
    print("    - max: ", np.max(data))
    print("    - min: ", np.min(data))

    return

def dotPlot(ax, idx, data, colors):
    print("  - Circle coverage: ", data[0])
    ax.plot(idx-0.3+ 1.0 * idx + 1.0, data[0],  marker='D', linestyle='dashed', markeredgecolor=colors, markerfacecolor=colors)
    return

with open(sys.argv[1]) as file:
    # The FullLoader parameter handles the conversion from YAML
    # scalar values to Python the dictionary format
    list = yaml.load(file, Loader=yaml.FullLoader)

    dataset_circle_goal = {}
    dataset_yaw_goal = {}

    idx = 0

    fig = plt.figure("Coverage", figsize=(5, 3))
    ax = fig.add_subplot(1, 1, 1)

    fig2 = plt.figure("Coverage Boxplot", figsize=(5, 3.5))
    ax2 = fig2.add_subplot(1, 1, 1)
    colors = ['c', 'm']

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
                circle_goal, yaw_goal = visualizeCoverage(ax, name, data_df)

                dataset = [yaw_goal]
                boxPlot(ax2, idx, dataset, [colors[1]])
                dotPlot(ax2, idx, circle_goal, colors[0])
                idx = idx + 1

                continue
    
    ax.set_xlabel('Yaw [rad]')
    ax.set_ylabel('Coverage')
    # ax.set_ylim([0.0, 1.0])
    ax.grid(True)
    ax.legend(loc="lower right")



    custom_lines = [plt.Line2D([0], [0], color=colors[0], lw=4),
                    plt.Line2D([0], [0], color=colors[1], lw=4)]

    ax2.legend(custom_lines, ['Valid Loiter Position', 'Tangential Loiter Position'], loc='lower left')
    ax2.set_ylabel('Coverage')
    # ax.set_ylim([0.0, 1.0])
    ax2.grid(True)

    plt.tight_layout()
    plt.show()
