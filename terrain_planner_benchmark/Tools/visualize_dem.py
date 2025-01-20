#!/usr/bin/env python3

from osgeo import gdal
from osgeo import ogr
import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np
import argparse
import matplotlib.image as mpimg
from mpl_toolkits.axes_grid1 import make_axes_locatable
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import pandas as pd


def visualizeDEM2D(fig, dataset, path_df):
    ax = fig.add_subplot(2, 1, 1)

    band = dataset.GetRasterBand(1)
    arr = band.ReadAsArray()

    path_x = path_df["x"].to_numpy()
    path_y = path_df["y"].to_numpy()

    # cax = fig.add_axes()
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    im = ax.imshow(np.flip(arr.T, axis=1), origin='lower')
    cb = fig.colorbar(im, cax=cax, orientation='vertical')
    cb.set_label('Altitude [m]')
    # ax.set_ylabel("Y [m]")
    # ax.plot_surface(X, Y, arr, cmap=cm.coolwarm, linewidth=0, antialiased=False, alpha=alpha_value)
    ax.plot(path_y/10, path_x/10, 'c-')
    ax.tick_params(axis='both',          # changes apply to the x-axis
        which='both',      # both major and minor ticks are affected
        bottom=False,      # ticks along the bottom edge are off
        top=False,         # ticks along the top edge are off
        left = False, right = False,
        labelbottom=False,
        labelleft = False)

def visualizeDEM3D(fig, dataset, path_df):

    ax = fig.add_subplot(1, 1, 1, projection='3d')
    # ax.set_aspect('equal')
    ax.view_init(elev=40., azim=180)
    band = dataset.GetRasterBand(1)
    arr = band.ReadAsArray()

    transform  = dataset.GetGeoTransform()

    xres = transform[1]
    yres = transform[5]

    # X = np.arange(transform[0], transform[0] + arr.shape[1]*xres, xres)
    # Y = np.arange(transform[3], transform[3] + arr.shape[0]*yres, yres)
    X = np.arange(0.0, 0.0 + arr.shape[1]*xres, xres)
    Y = np.arange(0.0, 0.0 + arr.shape[0]*yres, yres)
    # Shift DEM
    Y = Y + np.abs(np.min(Y))
    X, Y = np.meshgrid(X, Y)

    path_x = path_df["x"].to_numpy()
    path_y = path_df["y"].to_numpy()
    path_z = path_df["z"].to_numpy()

    # cax = fig.add_axes()
    # divider = make_axes_locatable(ax)
    # cax = divider.append_axes("right", size="5%", pad=0.05)
    # im = ax.imshow(np.flip(arr.T, axis=1), origin='lower')
    # cb = fig.colorbar(im, cax=cax, orientation='vertical')
    # cb.set_label('Altitude [m]')
    ax.plot_surface(X, Y, arr, cmap=cm.coolwarm, linewidth=0, antialiased=False, alpha=0.7, edgecolor='none')

    # ax.plot3D(path_x, path_y, path_z, 'c-')
    cmap = cm.get_cmap('gist_rainbow')
    points = np.array([path_x, path_y, path_z]).T.reshape(-1,1,3)
    segments = np.concatenate([points[:-1], points[1:]], axis = 1)
    intensity = (path_z - np.min(path_z)) / np.ptp(path_z)
    colors = [cmap(k) for k in intensity[:-1]]
    line_segments = Line3DCollection(segments, colors=colors, linewidth=2)
    ax.add_collection3d(line_segments)

    ax.set_box_aspect((np.ptp(X), np.ptp(Y), np.ptp(arr)))
    # ax.tick_params(axis='both',          # changes apply to the x-axis
    #     which='both',      # both major and minor ticks are affected
    #     bottom=False,      # ticks along the bottom edge are off
    #     top=False,         # ticks along the top edge are off
    #     left = False, right = False,
    #     labelbottom=False,
    #     labelleft = False)

def visualizeDEM3DMayavi(fig, dataset, path_df):

    ax = fig.add_subplot(2, 1, 1)
    # ax.set_aspect('equal')
    # ax.view_init(elev=40., azim=180)
    band = dataset.GetRasterBand(1)
    arr = band.ReadAsArray()

    transform  = dataset.GetGeoTransform()

    xres = transform[1]
    yres = transform[5]

    # X = np.arange(transform[0], transform[0] + arr.shape[1]*xres, xres)
    # Y = np.arange(transform[3], transform[3] + arr.shape[0]*yres, yres)
    X = np.arange(0.0, 0.0 + arr.shape[1]*xres, xres)
    Y = np.arange(0.0, 0.0 + arr.shape[0]*yres, yres)
    # Shift DEM
    Y = Y + np.abs(np.min(Y))
    X, Y = np.meshgrid(X, Y)

    path_x = path_df["x"].to_numpy()
    path_y = path_df["y"].to_numpy()
    path_z = path_df["z"].to_numpy()


    # cax = fig.add_axes()
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    im = ax.imshow(np.flip(arr.T, axis=1), origin='lower')
    cb = fig.colorbar(im, cax=cax, orientation='vertical')
    cb.set_label('Altitude [m]')
    # ax.set_ylabel("Y [m]")
    # cax = fig.add_axes()
    # divider = make_axes_locatable(ax)
    # cax = divider.append_axes("right", size="5%", pad=0.05)
    # im = ax.imshow(np.flip(arr.T, axis=1), origin='lower')
    # cb = fig.colorbar(im, cax=cax, orientation='vertical')
    # cb.set_label('Altitude [m]')

    # ax.plot3D(path_x, path_y, path_z, 'c-')
    cmap = cm.get_cmap('gist_rainbow')
    points = np.array([path_x, path_y, path_z]).T.reshape(-1,1,3)
    segments = np.concatenate([points[:-1], points[1:]], axis = 1)
    intensity = (path_z - np.min(path_z)) / np.ptp(path_z)
    colors = [cmap(k) for k in intensity[:-1]]
    line_segments = Line3DCollection(segments, colors=colors, linewidth=2)

    # ax.tick_params(axis='both',          # changes apply to the x-axis
    #     which='both',      # both major and minor ticks are affected
    #     bottom=False,      # ticks along the bottom edge are off
    #     top=False,         # ticks along the top edge are off
    #     left = False, right = False,
    #     labelbottom=False,
    #     labelleft = False)

    from mayavi import mlab
    from matplotlib.cm import get_cmap # for viridis
    
    # su = mlab.surf(X.T, Y.T, arr.T)
    # su = mlab.mesh(X.T, Y.T, arr.T)
    fig = mlab.figure(bgcolor=(1,1,1), size=(1220,748))
    # fig = mlab.figure(bgcolor=(1,1,1), size=(1220,600))
    su = mlab.mesh(X.T, Y.T, arr.T, representation='surface')

    # manually set viridis for the surface
    cmap_name = 'viridis'
    cdat = np.array(get_cmap(cmap_name,256).colors)
    cdat = (cdat*255).astype(int)
    su.module_manager.scalar_lut_manager.lut.table = cdat

    mlab.plot3d(path_x, path_y, path_z, tube_radius=20.025, color=(1,0,0))
    
    # f = mlab.gcf()
    # f.scene._lift()
    # arr = mlab.screenshot()
    # ax.imshow(arr)
    mlab.yaw(180.0)
    mlab.view(azimuth=180.0, elevation=50.0, distance=17000, focalpoint=None,
        roll=None, reset_roll=True, figure=None)
    # mlab.show()


def visualizeICS(fig, ax, dataset):
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    im = ax.imshow(dataset, cmap=cm.get_cmap('winter', 2), origin='upper')
    cb = fig.colorbar(im, cax=cax, orientation='vertical')
    # cb.remove()
    ax.set_xlabel("X [km]")
    ax.set_xticklabels(['', '0.0', '2.0', '4.0', '6.0', '8.0', '10.0', '12.0'])
    cb.ax.get_yaxis().set_ticks([])
    for j, lab in enumerate(['Invalid', 'Valid']):
        cb.ax.text(0.6, 0.5*j + 0.25, lab, ha='center', va='center', rotation=90)
    # ax.set_ylabel("Y [m]")

# location='dischma_valley'
# location='sargans'
location='gotthard'


# initialize the graph:
reference_dataset = gdal.Open('/home/jaeyoung/catkin_ws/src/terrain-models/models/' + location + '.tif')
ics_dataset = mpimg.imread('/home/jaeyoung/catkin_ws/src/terrain-navigation/output/' + location + '_circle_goal.png')
path_df = pd.read_csv('/home/jaeyoung/catkin_ws/src/terrain-navigation/output/' + location + '_planned_path.csv')

path_x = path_df["x"].to_numpy()
path_y = path_df["y"].to_numpy()
path_z = path_df["z"].to_numpy()

fig1 = plt.figure("Terrain and Path Visualization", figsize=(4.0, 2.7))

## 2D visualization of planned path
# visualizeDEM2D(fig1, reference_dataset, path_df)

ax2 = fig1.add_subplot(1, 1, 1)
visualizeICS(fig1, ax2, ics_dataset)
# ax2.plot(1220 - path_y/10, 742.0 - path_x/10, 'r-')
ax2.tick_params(axis='both',          # changes apply to the x-axis
    which='both',      # both major and minor ticks are affected
    # bottom=False,      # ticks along the bottom edge are off
    top=False,         # ticks along the top edge are off
    left = False, right = False,
    # labelbottom=False,
    labelleft = False)

# ax2.set_title('Mask')
fig2 = plt.figure("Dummy", figsize=(5, 5.8))
# visualizeDEM3DMayavi(fig2, reference_dataset, path_df)


plt.tight_layout()
plt.show()

print("\n")
print("Done")
