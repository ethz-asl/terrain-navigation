#!/usr/bin/env python3
# This python script visualizes the geometric priors
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys
import yaml

def loadDataFrames(path):
    data_df = pd.read_csv(path)
    return data_df

def analyzeErrorStatistics(fig, data_df, name):
    error = np.array(data_df["error"])
    n, bins, patches = fig.hist(error, bins=100, alpha=0.75, label=name)
    fig.set_xlabel('Error [m]')
    fig.set_ylabel('Number of Points')
    # fig.set_xlim([0.0, 3.0])
    fig.set_title('Error')
    fig.grid(True)
    fig.legend(loc="upper right")

def analyzeUtilityStatistics(fig, data_df, name):
    error = np.array(data_df["utility"])
    n, bins, patches = fig.hist(error, bins=100, alpha=0.75, label=name)
    fig.set_xlabel('Cramer Rao Bounds [m]')
    fig.set_ylabel('Number of Points')
    fig.set_title('Cramer Rao Bounds')
    fig.grid(True)
    fig.legend(loc="upper right")

def analyzePrecision(fig, data_df, name):
    error = np.array(data_df["error"])

    total = np.count_nonzero(~np.isnan(error))
    precision = np.array([])
    length = np.arange(0.0, 5.0, 0.05)
    for threshold in length:
        rate = (error < threshold).sum() / total
        precision = np.append(precision, rate)

    fig.plot(length, precision, '-', label=name)
    fig.set_xlabel('Error [m]')
    fig.set_ylabel('Precision')
    fig.set_title('Precision')
    fig.grid(True)
    fig.legend(loc="upper right")

fig1 = plt.figure("Error Statistics")
#Error Histogram
ax11 = fig1.add_subplot(1, 1, 1)

fig2 = plt.figure("Utility Statistics")
ax21 = fig2.add_subplot(1, 1, 1)

fig3 = plt.figure("Precision Curve")
ax31 = fig3.add_subplot(1, 1, 1)

with open(sys.argv[1]) as file:
    # The FullLoader parameter handles the conversion from YAML
    # scalar values to Python the dictionary format
    list = yaml.load(file, Loader=yaml.FullLoader)
    for key, value in list.items():
        print("Plotting ", key)
        path = value['path']
        print("  Reading: ", path)
        map_data_df = pd.read_csv(value['path'])
        analyzeErrorStatistics(ax11, map_data_df, value['name'])
        analyzeUtilityStatistics(ax21, map_data_df, value['name'])
        analyzePrecision(ax31, map_data_df, value['name'])
plt.legend()
plt.show()
