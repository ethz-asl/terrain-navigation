#!/usr/bin/env python3
# This python script visualizes the geometric priors
import matplotlib.pyplot as plt
import numpy as np
import math
import pandas as pd
import sys

def analyzeMapStatistics(data_df):
    experiment_id = np.array(data_df["id"])
    error = np.array(data_df["error"])
    utility = np.array(data_df["utility"])
    gsd = np.array(data_df["ground_sample_distance"])
    # the histogram of the data

    fig = plt.figure("Map Statistics")
    ax1 = fig.add_subplot(1, 3, 1)
    n, bins, patches = ax1.hist(error, bins=50, facecolor='g', alpha=0.75)
    ax1.set_xlabel('Error [m]')
    ax1.set_ylabel('Number of Points')
    ax1.set_title('Error')
    ax1.grid(True)

    ax2 = fig.add_subplot(1, 3, 2)
    n, bins, patches = ax2.hist(utility, bins=50, facecolor='g', alpha=0.75)
    ax2.set_xlabel('Utility []')
    ax2.set_xlim([0.5, 1.0])
    ax2.set_ylabel('Number of Points')
    ax2.set_title('View Utility')
    ax2.grid(True)

    ax3 = fig.add_subplot(1, 3, 3)
    n, bins, patches = ax3.hist(gsd, bins=50, facecolor='g', alpha=0.75)
    ax3.set_xlabel('Resolution Prior')
    ax3.set_xlim([0.5, 1.0])
    ax3.set_ylabel('Number of Points')
    ax3.set_title('Resolution Prior')
    ax3.grid(True)

    fig2 = plt.figure("Map Scatter Plot")
    ax3 = fig2.add_subplot(1, 1, 1)
    ax3.scatter(utility, error, alpha=0.5)
    ax3.set_xlabel('View Utility')
    ax3.set_ylabel('Error')
    plt.show()



# Planner benchmark
map_data_df = pd.read_csv(sys.argv[1])

analyzeMapStatistics(map_data_df)
