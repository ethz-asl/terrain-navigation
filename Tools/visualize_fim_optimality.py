#!/usr/bin/env python3
# This python script visualizes the geometric priors
import matplotlib.pyplot as plt
import numpy as np
import math
import pandas as pd
import sys
from mpl_toolkits.axes_grid1 import make_axes_locatable
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def getFIM(view_vector, res) :
    distance = np.linalg.norm(view_vector)
    Jacobian =  1/distance * np.identity(3) - (1/np.power(distance, 3)) * np.outer(view_vector, view_vector)
    # HandJacobian = (1/np.power(distance, 3)) * np.array([[distance**2 - view_vector[0]**2, -view_vector[0] *view_vector[1], -view_vector[0] *view_vector[2]],
    #                          [-view_vector[0] *view_vector[1], distance**2 - view_vector[1]**2, -view_vector[1] *view_vector[2]],
    #                          [-view_vector[0] *view_vector[2], -view_vector[1] *view_vector[2], distance**2 - view_vector[2]**2]])
    pixel_res = 0.5 * math.pi / res# Angle resolution of a single pixel
    sigma = math.sin(pixel_res) # Angle resolution of a single pixel
    FIM = (1/np.power(sigma, 2)) * np.matmul(np.transpose(Jacobian), Jacobian)
    return FIM

def getEopt(FIM) :
    w, v = np.linalg.eig(FIM)
    return np.min(w)

def getDopt(FIM) :
    return np.linalg.det(FIM)

def getDopt(FIM) :
    return np.linalg.det(FIM)


def plotCramerRaoBounds(ax, res, label):

    accumulated_fim = np.zeros((3,3))
    bound = np.array([])
    views = np.array(range(20)) + 1
    distance = 100

    for i in views:
        #TODO: Keep track of statistics
        rand_bearing = np.random.rand(3)
        rand_bearing = distance * rand_bearing / np.linalg.norm(rand_bearing)
        fim = getFIM(rand_bearing, res)
        accumulated_fim = np.add(accumulated_fim, fim)
        cramer_rao_bound = np.diag(np.linalg.inv(accumulated_fim))
        min_bound = np.sqrt(max(cramer_rao_bound))
        bound = np.append(bound, min_bound)
    
    ax.plot(views[np.abs(bound) < 1.0], bound[np.abs(bound) < 1.0], '-o', label=label)
    ax.legend(loc="upper right")

def viewpointFIM(x, y, view):
    nadir = np.array([0.0, 0.0, -1.0])
    bearing = np.array([x, y, 0.0]) - view
    bearing_u = bearing / np.linalg.norm(bearing)
    phi = math.acos(np.dot(bearing_u, nadir))
    sigma_k = math.pi * 30.0 / 180.0
    v = math.exp(-0.5 * math.pow(phi/sigma_k, 2))
    fim = getFIM(bearing, 1440)
    return v * fim

def calculateCramerRaoBounds(x, y):
    max_bound = 1.0
    view1 = np.array([0.0, 0.0, 100.0])
    view2 = np.array([0.0, 50.0, 100.0])
    if (x > 100.0 or x < -100) or (y > 100 or y < -50):
        max_bound = float('nan')
    else:
        fim1 = viewpointFIM(x, y, view1)
        fim2 = viewpointFIM(x, y, view2)

        accumulated_fim = fim1 + fim2
        cramer_rao_bound = np.diag(np.linalg.inv(accumulated_fim))
        max_bound = np.sqrt(max(cramer_rao_bound))
        # max_bound = min(max_bound, 1.0)
    return max_bound

def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

def getViewVertices(view):
    v = np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1],  [-1, 1, -1], [0, 0, 0]])
    v = 10.0*v + view
    # generate list of sides' polygons of our pyramid
    verts = [ [v[0],v[1],v[4]], [v[0],v[3],v[4]],
    [v[2],v[1],v[4]], [v[2],v[3],v[4]], [v[0],v[1],v[2],v[3]]]
    return verts

def visualizeCramerRaoBounds(ax):
    x = np.linspace(-100, 100, num=200)
    y = np.linspace(-100, 150, num=250)
    view1 = np.array([0.0, 0.0, 100.0])
    view2 = np.array([0.0, 50.0, 100.0])
    # vertices of a pyramid

    # plot sides
    ax.add_collection3d(Poly3DCollection(getViewVertices(view1), linewidths=1, edgecolors=[0, 0, 1], alpha=.25))
    ax.add_collection3d(Poly3DCollection(getViewVertices(view2), linewidths=1, edgecolors='r', alpha=.25))
    xx, yy = np.meshgrid(x, y)
    vfunc = np.vectorize(calculateCramerRaoBounds)
    zz = vfunc(xx, yy)

    # h = plt.contourf(x, y, zz)
    # ax.plot_surface(xx, yy, zz, rstride=8, cstride=8, alpha=0.3)
    view1_fov_x = np.array([100, -100, -100, 100, 100])
    view1_fov_y = np.array([150, 150, -50, -50, 150])
    view1_fov_z = np.array([0, 0, 0, 0, 0])
    ax.plot3D(view1_fov_x, view1_fov_y, view1_fov_z, 'r-', zorder=20)

    view2_fov_x = np.array([100, -100, -100, 100, 100])
    view2_fov_y = np.array([100, 100, -100, -100, 100])
    view2_fov_z = np.array([0, 0, 0, 0, 0])
    ax.plot3D(view2_fov_x, view2_fov_y, view2_fov_z, 'b-', zorder=20)

    cset = ax.contourf(xx, yy, zz, zdir='z', offset=0, levels=10, cmap='viridis')
    cb = fig.colorbar(cset, ax = ax, shrink=0.6)
    cb.set_label('Cramer-Rao Bounds [m]')
    ax.scatter(view1[0], view1[1], view1[2])
    ax.scatter(view2[0], view2[1], view2[2])
    ax.set_zlim([0.0, 100.0])
    ax.set_box_aspect([2,2.5,1])

fig = plt.figure("Information", figsize=(4, 2))
# ax1 = fig.add_subplot(1, 3, 1)
# ax1.plot(information_e)
# ax1.set_title("E-Optimality")
# ax1.set_xlabel("Number of views")

# ax2 = fig.add_subplot(1, 3, 2)
# ax2.plot(information_d)
# ax2.set_title("D-Optimality")
# ax2.set_xlabel("Number of views")

ax3 = fig.add_subplot(1, 1, 1)
# plotCramerRaoBounds(ax3, 1440, "1440 X 1080")
# plotCramerRaoBounds(ax3, 720, "720 X 540")
# plotCramerRaoBounds(ax3, 360, "360 X 270")
# plotCramerRaoBounds(ax3, 200, "300m")

ax3.set_title("Cramer Rao Bounds [m]")
ax3.set_xlabel("Number of views")
ax3.set_ylabel("Cramer Rao Bounds [m]")
ax3.set_xticks(range(1, 11, 1))
ax3.set_xlim([1, 10])
ax3.grid()

fig2 = plt.figure("CRBounds 3D", figsize=(6, 4))
ax4 = fig2.add_subplot(111, projection='3d')
visualizeCramerRaoBounds(ax4)
ax4.set_xlabel("X[m]")
ax4.set_ylabel("Y[m]")
ax4.set_zlabel("Z[m]")

# ax4.set_zlabel("Z[m]")
# plt.colorbar()
plt.tight_layout()
plt.show()
