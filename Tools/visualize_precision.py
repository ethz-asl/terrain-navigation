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

def getPrecision(data_df, threshold):
    error = np.array(data_df["error"])
    total = np.count_nonzero(~np.isnan(error))
    precision = (error < threshold).sum() / total
    return precision

def getCompleteness(data_df, threshold):
    error = np.array(data_df["error"])
    total = np.prod(error.shape)
    completeness = (error < threshold).sum() / total
    return completeness


fig1 = plt.figure("Error Statistics", figsize=(6, 4))
#Error Histogram
ax11 = fig1.add_subplot(2, 1, 1)
ax11.set_ylim([0.0, 1.0])
ax11.set_ylabel('Precision')
ax11.set_xlabel('Number of Views')
ax11.grid(True)
ax11.legend(loc="lower right")
ax11.set_xticks(range(10, 110, 10))

ax12 = fig1.add_subplot(2, 1, 2)
ax12.set_ylim([0.0, 1.0])
ax12.set_ylabel('Completeness')
ax12.set_xlabel('Number of Views')
ax12.grid(True)
ax12.legend(loc="lower right")
ax12.set_xticks(range(10, 110, 10))

max_error = 2.0

with open(sys.argv[1]) as file:
    # The FullLoader parameter handles the conversion from YAML
    # scalar values to Python the dictionary format
    list = yaml.load(file, Loader=yaml.FullLoader)
    for category_name, category in list.items():
        views = np.array([])
        precision = np.array([])
        completeness = np.array([])
        name = {}
        for key, value in category.items():
            print("key: ", key, " value: ", value)
            if key == 'name':
                name = value
                continue
            views = np.append(views, value["num_views"])

            map_data_df = pd.read_csv(value['path'])

            precision = np.append(precision, getPrecision(map_data_df, max_error))
            completeness = np.append(completeness, getCompleteness(map_data_df, max_error))

        ax11.plot(views, precision, '-o', label=name)
        ax11.legend(loc="lower right")
        ax12.plot(views, completeness, '-o', label=name)
        ax12.legend(loc="lower right")
plt.tight_layout()
plt.legend()
plt.show()
