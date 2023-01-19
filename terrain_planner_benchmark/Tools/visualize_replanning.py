#!/usr/bin/env python3
# This python script visualizes the geometric priors
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys
import yaml
import os


def appendDataset(data_df):
    # circle_goal_path_length = getResults(data_df, "circle_goal")/1000.0 # convert m to km
    # yaw_goal_path_length = getResults(data_df, "yaw_goal")/1000.0 # convert m to km
    # return circle_goal_path_length, yaw_goal_path_length
    return

def main():
    directory=sys.argv[1]
    # The FullLoader parameter handles the conversion from YAML
    # scalar values to Python the dictionary format

    fig = plt.figure("Coverage")
    ax = fig.add_subplot(1, 1, 1)
    colors = ['pink', 'lightblue']

    dataset_circle_goal = {}
    dataset_yaw_goal = {}

    idx = 0

    for filename in os.listdir(directory):
        f = os.path.join(directory, filename)
        # checking if it is a file
        if os.path.isfile(f):
            #TODO If file name matches a pattern
            appendDataset(pd.read_csv(f))

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()