#!/usr/bin/env python3
# This python script visualizes the geometric priors
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys
import yaml
import os
import fnmatch
from cycler import cycler


def appendDataset(ax, data_df):
    x = np.array(data_df["x"])
    y = np.array(data_df["y"])
    z = np.array(data_df["z"])

    ax.plot3D(x, y, z)
    # circle_goal_path_length = getResults(data_df, "circle_goal")/1000.0 # convert m to km
    # yaw_goal_path_length = getResults(data_df, "yaw_goal")/1000.0 # convert m to km
    # return circle_goal_path_length, yaw_goal_path_length
    return

def main():
    directory=sys.argv[1]
    # The FullLoader parameter handles the conversion from YAML
    # scalar values to Python the dictionary format
    n = 5 # Number of colors
    new_colors = [plt.get_cmap('Blues')(1. * (i+2)/(n+2)) for i in range(n)]

    plt.rc('axes', 
        prop_cycle=(cycler('color', new_colors)))

    fig = plt.figure("Coverage")
    ax = plt.axes(projection='3d')

    for filename in os.listdir(directory):
        f = os.path.join(directory, filename)
        if os.path.isfile(f): # checking if it is a file
            if fnmatch.fnmatch(filename, '*solution_*'):
                print(f)
                appendDataset(ax, pd.read_csv(f))
    ax.set_box_aspect([3,4,0.5])
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
