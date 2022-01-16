#!/usr/bin/env python3
# This python script visualizes the geometric priors
import matplotlib.pyplot as plt
import numpy as np
import math
import pandas as pd
import sys
from scipy import stats

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
    n, bins, patches = ax1.hist(error, bins=100, facecolor='g', alpha=0.75)
    ax1.set_xlabel('Error [m]')
    ax1.set_ylabel('Number of Points')
    ax1.set_title('Error')
    ax1.grid(True)

    ax2 = fig.add_subplot(2, 2, 2)
    n, bins, patches = ax2.hist(utility, bins=100, facecolor='g', alpha=0.75)
    ax2.set_xlabel('Map Quality []')
    ax2.set_ylabel('Number of Points')
    ax2.set_title('Map Quality')
    ax2.grid(True)

    ax3 = fig.add_subplot(2, 2, 3)
    n, bins, patches = ax3.hist(gsd, bins=100, facecolor='g', alpha=0.75)
    ax3.set_xlabel('Resolution Prior')
    ax3.set_xlim([0.0, 1.0])
    ax3.set_ylabel('Number of Points')
    ax3.set_title('Resolution Prior')
    ax3.grid(True)

    ax4 = fig.add_subplot(2, 2, 4)
    n, bins, patches = ax4.hist(visibility, bins=100, facecolor='g', alpha=0.75)
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
    min_eigen_value = np.array(data_df["min_eigen_value"])

    fig2 = plt.figure("Map Scatter Plot")
    ax4 = fig2.add_subplot(3, 2, 1)
    ax4.scatter(utility, error, alpha=0.2, s=1.0)
    ax4.set_xlabel('Cell Quality')
    # ax4.set_xlim([0.0, 1.0])
    ax4.set_ylabel('Error [m]')

    ax5 = fig2.add_subplot(3, 2, 2)
    ax5.scatter(gsd[utility > 0], error[utility > 0], alpha=0.5, s=0.5)
    ax5.set_xlabel('Resolution Prior')
    ax5.set_ylabel('Error [m]')

    ax6 = fig2.add_subplot(3, 2, 3)
    ax6.scatter(incident, error, alpha=0.5, s=0.5)
    ax6.set_xlabel('incident')
    ax6.set_ylabel('Error [m]')

    ax7 = fig2.add_subplot(3, 2, 4)
    ax7.scatter(triangulation[utility > 0], error[utility > 0], alpha=0.5, s=0.5)
    ax7.set_xlabel('triangulation')
    ax7.set_ylabel('Error [m]')

    ax8 = fig2.add_subplot(3, 2, 6)
    ax8.scatter(min_eigen_value, error, alpha=0.5, s=0.5)
    ax8.set_xlabel('Eigen Value')
    ax8.set_ylabel('Error [m]')

    ax9 = fig2.add_subplot(3, 2, 5)
    ax9.scatter(visibility, error, alpha=0.5, s=0.5)
    ax9.set_xlabel('Visibility Count')
    ax9.set_ylabel('Error [m]')

def analyzePrecision(data_df):
    error = np.array(data_df["error"])
    utility = np.array(data_df["utility"])

    total = np.count_nonzero(~np.isnan(error))
    precision = np.array([])
    length = np.arange(0.0, 5.0, 0.5)
    for threshold in length:
        rate = (error < threshold).sum() / total
        precision = np.append(precision, rate)

    fig3 = plt.figure("Map Precision")
    ax = fig3.add_subplot(1,1,1)
    ax.plot(length, precision, '-o')
    ax.set_xlabel('Error [m]')
    ax.set_ylabel('Precision')
    ax.set_title('Precision')
    ax.grid(True)

def makeupMetrics(data_df):
    error = np.array(data_df["error"])
    utility = np.array(data_df["utility"])
    gsd = np.array(data_df["ground_sample_distance"])
    incident = np.array(data_df["incident_prior"])
    triangulation = np.array(data_df["triangulation_prior"])
    visibility = np.array(data_df["visibility"])
    min_eigen_value = np.array(data_df["min_eigen_value"])

    fig3 = plt.figure("New Scatter Plot")
    ax31 = fig3.add_subplot(1, 1, 1)
    ax31.scatter(utility[utility>0.0], error[utility>0.0], alpha=0.2, s=0.5)
    ax31.set_xlabel('Maximum Cramer Rao Bound [m]')
    ax31.set_ylabel('Error [m]')

    bin_means, bin_edges, binnumber = stats.binned_statistic(utility[~np.isnan(utility)], error[~np.isnan(utility)],
            statistic='std', bins=1000)
    bin_width = (bin_edges[1] - bin_edges[0])
    bin_centers = bin_edges[1:] - bin_width/2
    # plt.hist(samples, bins=50, density=True, histtype='stepfilled',
    #         alpha=0.2, label='histogram of data')
    ax31.hlines(bin_means, bin_edges[:-1], bin_edges[1:], colors='g', lw=3,
            label='binned statistic of data')
    # ax31.plot((binnumber - 0.5) * bin_width, bin_means, 'g.', alpha=0.5)
# Planner benchmark
map_data_df = pd.read_csv(sys.argv[1])

analyzeMapStatistics(map_data_df)
analyzeMapScatter(map_data_df)
makeupMetrics(map_data_df)
analyzePrecision(map_data_df)
plt.show()
