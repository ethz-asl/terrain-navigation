#!/usr/bin/env python3
# This python script visualizes the geometric priors
import matplotlib.pyplot as plt
import numpy as np
import math
import pandas as pd
import sys
import yaml

def plotCoverage(fig, data_df, name):
    timestamp = np.array(data_df["timestamp"])
    map_coverage = np.array(data_df["coverage"])
    map_coverage_variance = np.sqrt(np.array(data_df["coverage_variance"]))
    fig.fill_between(timestamp, map_coverage - map_coverage_variance, map_coverage + map_coverage_variance, alpha=0.2)
    # TODO: Add plotting variance on coverage

    fig.plot(timestamp, map_coverage, label=name)
    fig.set_title("Map Coverage")
    fig.set_xlabel('Time [s]')
    fig.set_ylabel('Coverage')
    fig.legend(loc='lower right')
    fig.grid(True)

def plotQuality(fig, data_df, name):
    timestamp = np.array(data_df["timestamp"])
    map_quality = np.array(data_df["quality"])
    map_quality_variance = np.sqrt(np.array(data_df["quality_variance"]))

    # TODO: Add plotting quality on coverage

    fig.plot(timestamp, map_quality, label=name)
    fig.fill_between(timestamp, map_quality - map_quality_variance, map_quality + map_quality_variance, alpha=0.2)
    fig.set_title("Map Uncertainty")
    fig.set_xlabel('Time [s]')
    fig.set_ylabel('Uncertainty')
    fig.legend(loc='upper right')
    fig.grid(True)


figure1 = plt.figure("Map Data")
axis1 = figure1.add_subplot(1, 2, 1)
axis2 = figure1.add_subplot(1, 2, 2)


# Planner benchmark
with open(sys.argv[1]) as file:
    # The FullLoader parameter handles the conversion from YAML
    # scalar values to Python the dictionary format
    list = yaml.load(file, Loader=yaml.FullLoader)
    for key, value in list.items():
        data_df = pd.read_csv(value['path'])
        plotCoverage(axis1, data_df, value['name'])
        plotQuality(axis2, data_df, value['name'])

plt.show()
