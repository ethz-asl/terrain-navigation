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
    incident = np.array(data_df["incident_prior"])
    triangulation = np.array(data_df["triangulation_prior"])
    visibility = np.array(data_df["visibility"])

    # the histogram of the data

    fig = plt.figure("Map Statistics")
    ax1 = fig.add_subplot(2, 2, 1)
    n, bins, patches = ax1.hist(error, bins=50, facecolor='g', alpha=0.75)
    ax1.set_xlabel('Error [m]')
    ax1.set_ylabel('Number of Points')
    ax1.set_title('Error')
    ax1.grid(True)

    ax2 = fig.add_subplot(2, 2, 2)
    n, bins, patches = ax2.hist(utility, bins=50, facecolor='g', alpha=0.75)
    ax2.set_xlabel('Map Quality []')
    ax2.set_xlim([0.1, 1.0])
    ax2.set_ylabel('Number of Points')
    ax2.set_title('Map Quality')
    ax2.grid(True)

    ax3 = fig.add_subplot(2, 2, 3)
    n, bins, patches = ax3.hist(gsd, bins=50, facecolor='g', alpha=0.75)
    ax3.set_xlabel('Resolution Prior')
    ax3.set_xlim([0.1, 1.0])
    ax3.set_ylabel('Number of Points')
    ax3.set_title('Resolution Prior')
    ax3.grid(True)

    ax4 = fig.add_subplot(2, 2, 4)
    n, bins, patches = ax4.hist(visibility, bins=50, facecolor='g', alpha=0.75)
    ax4.set_xlabel('visibility')
    # ax4.set_xlim([0.1, 1.0])
    ax4.set_ylabel('Number of Points')
    ax4.set_title('visibility')
    ax4.grid(True)

def analyzeMapScatter(data_df):
    error = np.array(data_df["error"])
    utility = np.array(data_df["utility"])
    gsd = np.array(data_df["ground_sample_distance"])
    incident = np.array(data_df["incident_prior"])
    triangulation = np.array(data_df["triangulation_prior"])
    visibility = np.array(data_df["visibility"])

    fig2 = plt.figure("Map Scatter Plot")
    ax4 = fig2.add_subplot(2, 2, 1)
    ax4.scatter(utility, visibility, alpha=0.5, s=0.5)
    ax4.set_xlabel('utility')
    ax4.set_ylabel('visibility')

    ax5 = fig2.add_subplot(2, 2, 2)
    ax5.scatter(gsd, error, alpha=0.5, s=0.5)
    ax5.set_xlabel('Resolution Prior')
    ax5.set_ylabel('Error')

    ax6 = fig2.add_subplot(2, 2, 3)
    ax6.scatter(incident, error, alpha=0.5, s=0.5)
    ax6.set_xlabel('incident')
    ax6.set_ylabel('Error')

    ax7 = fig2.add_subplot(2, 2, 4)
    ax7.scatter(visibility, error, alpha=0.5, s=0.5)
    ax7.set_xlabel('triangulation')
    ax7.set_ylabel('Error')


    plt.show()



# Planner benchmark
map_data_df = pd.read_csv(sys.argv[1])

analyzeMapStatistics(map_data_df)
analyzeMapScatter(map_data_df)