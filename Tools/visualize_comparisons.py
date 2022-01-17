#!/usr/bin/env python3
# This python script visualizes the geometric priors
import matplotlib.pyplot as plt
import numpy as np
import math
import pandas as pd
import sys
import yaml

def loadDataFrames(path):
    data_df = pd.read_csv(path)
    return data_df

def analyzeMapStatistics(fig, data_df):
    error = np.array(data_df["error"])
    n, bins, patches = ax1.hist(error, bins=100, facecolor='g', alpha=0.75)
    fig.set_xlabel('Error [m]')
    fig.set_ylabel('Number of Points')
    fig.set_title('Error')
    fig.grid(True)


fig1 = plt.figure("Error Statistics")
ax1 = fig1.add_subplot(1, 1, 1)


with open('resources/benchmark.yaml') as file:
    # The FullLoader parameter handles the conversion from YAML
    # scalar values to Python the dictionary format
    list = yaml.load(file, Loader=yaml.FullLoader)
    for key, value in list.items():
        print("Plotting ", key)
        path = value['path']
        print("  Reading: ", path)
        map_data_df = pd.read_csv(value['path'])
        analyzeMapStatistics(ax1, map_data_df)

plt.show()
