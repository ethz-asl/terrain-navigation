#!/usr/bin/env python3

from osgeo import gdal
from osgeo import ogr
import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np
import argparse
import matplotlib.image as mpimg
from mpl_toolkits.axes_grid1 import make_axes_locatable

def visualizeDEM(fig, ax, dataset):
    band = dataset.GetRasterBand(1)
    arr = band.ReadAsArray()

    # cax = fig.add_axes()
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    im = ax.imshow(arr.T)
    cb = fig.colorbar(im, cax=cax, orientation='vertical')
    cb.set_label('Altitude [m]')
    # ax.plot_surface(X, Y, arr, cmap=cm.coolwarm, linewidth=0, antialiased=False, alpha=alpha_value)

def visualizeICS(fig, ax, dataset):
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    im = ax.imshow(dataset, cmap=cm.get_cmap('cividis', 2), origin='lower')
    cb = fig.colorbar(im, cax=cax, orientation='vertical')
    # cb.remove()


# initialize the graph:
reference_dataset = gdal.Open('/home/jaeyoung/catkin_ws/src/terrain-models/models/dischma_valley.tif')
ics_dataset = mpimg.imread('/home/jaeyoung/catkin_ws/src/terrain-navigation/output/dischma_valley_circle_goal.png')

fig1 = plt.figure("Terrain and Path Visualization", figsize=(5, 5))
ax1 = fig1.add_subplot(2, 1, 1)

visualizeDEM(fig1, ax1, reference_dataset)
# ax1.set_title('Elevation')
# ax1.set_xlabel("X [m]")
# ax1.set_ylabel("Y [m]")
ax1.tick_params(axis='x',          # changes apply to the x-axis
    which='both',      # both major and minor ticks are affected
    bottom=False,      # ticks along the bottom edge are off
    top=False,         # ticks along the top edge are off
    labelbottom=False)

# ax1.set_zlabel("Z [m]")

ax2 = fig1.add_subplot(2, 1, 2)
visualizeICS(fig1, ax2, ics_dataset)
# ax2.set_title('Mask')
# ax2.set_xlabel("X [m]")
# ax2.set_ylabel("Y [m]")

plt.tight_layout()
plt.show()

print("\n")
print("Done")