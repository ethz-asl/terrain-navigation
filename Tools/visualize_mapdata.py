#!/usr/bin/env python3
# This python script visualizes the geometric priors
import matplotlib.pyplot as plt
import numpy as np
import math
import pandas as pd
import sys

def planner_benchmark(data_df):
    experiment_id = np.array(data_df["id"])
    error = np.array(data_df["error"])
    utility = np.array(data_df["utility"])
    # the histogram of the data

    fig = plt.figure("Map Data")
    ax1 = fig.add_subplot(1, 2, 1)
    n, bins, patches = ax1.hist(error, bins=50, facecolor='g', alpha=0.75)
    ax1.set_xlabel('Error [m]')
    ax1.set_ylabel('Number of Points')
    ax1.set_title('Error')
    ax1.grid(True)

    ax2 = fig.add_subplot(1, 2, 2)
    n, bins, patches = ax2.hist(utility, bins=50, facecolor='g', alpha=0.75)
    ax2.set_xlabel('Utility []')
    ax2.set_ylabel('Number of Points')
    ax2.set_title('View Utility')
    ax2.grid(True)
    plt.show()



# Planner benchmark
map_data_df = pd.read_csv(sys.argv[1])

planner_benchmark(map_data_df)