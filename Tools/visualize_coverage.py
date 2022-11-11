#!/usr/bin/env python3
# This python script visualizes the geometric priors
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys
import yaml

fig = plt.figure("Coverage")
ax = fig.add_subplot(1, 1, 1)

data_df = pd.read_csv('/home/jaeyoung/catkin_ws/src/terrain-navigation/output/output.csv')

yaw = np.array(data_df["yaw"])
yaw_coverage = np.array(data_df["yaw_coverage"])
circle_coverage = np.array(data_df["circle_coverage"])


ax.plot(yaw, yaw_coverage, '-', label='Goal Yaw')
ax.plot(yaw, circle_coverage, '--', label='Goal Circle')
ax.set_xlabel('Yaw [rad]')
ax.set_ylabel('Coverage')
ax.set_title('Valid Goal state coverage')
ax.grid(True)
ax.legend(loc="lower right")

plt.show()