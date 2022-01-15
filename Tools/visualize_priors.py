#!/usr/bin/env python3
# This python script visualizes the geometric priors
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import math


def wrap_pi(theta):
    if theta > math.pi:
        theta_before = theta
        theta = 2 * math.pi - theta
    return theta

def visualize_angular_priors():
    sigma = 45.0/ 180.0 * math.pi
    min_angle = 30 / 180.0 * math.pi
    # generate 2 2d grids for the x & y bounds
    y, x = np.meshgrid(np.linspace(0.0, 1, 100), np.linspace(-0.5, 0.5, 100))

    theta = np.abs(np.arctan(x/y))
    trian_prior = 1 - (np.minimum(theta, min_angle) - min_angle)**2 / min_angle **2 # Triangulation Prior
    inc_prior = np.exp(-theta**2 / sigma**2)

    z = np.multiply(trian_prior, inc_prior)

    # x and y are bounds, so z should be the value *inside* those bounds.
    # Therefore, remove the last value from the z array.
    z = z[:-1, :-1]
    z_min, z_max = 0.0, 1.0

    fig, ax = plt.subplots()
    c = ax.pcolormesh(x, y, z, cmap='Blues', vmin=z_min, vmax=z_max)
    ax.set_title('Anglular Prior')
    # set the limits of the plot to the limits of the data
    ax.axis([x.min(), x.max(), y.min(), y.max()])
    ax.set_aspect('equal', 'box')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('z [m]')
    fig.colorbar(c, ax=ax)

def visualize_distance_priors():
    reference_view_distance = 0.5
    gsd_view_distance = 0.7

    # generate 2 2d grids for the x & y bounds
    y, x = np.meshgrid(np.linspace(0.0, 1, 100), np.linspace(-0.5, 0.5, 100))

    dist = np.sqrt(x**2 + y**2)

    relative_area = (reference_view_distance / dist)**2
    resolution_prior = np.minimum(relative_area, 1/relative_area)
    relative_gsd = (gsd_view_distance / dist)**2
    gsd_prior = np.minimum(relative_gsd, 1/relative_gsd)

    z = np.multiply(resolution_prior, gsd_prior)

    # x and y are bounds, so z should be the value *inside* those bounds.
    # Therefore, remove the last value from the z array.
    z = z[:-1, :-1]
    z_min, z_max = 0.0, 1.0

    fig, ax = plt.subplots()

    c = ax.pcolormesh(x, y, z, cmap='Blues', vmin=z_min, vmax=z_max)
    ax.set_title('Distance Prior')
    # set the limits of the plot to the limits of the data
    ax.axis([x.min(), x.max(), y.min(), y.max()])
    ax.set_aspect('equal', 'box')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('z [m]')
    fig.colorbar(c, ax=ax)

def visualize_joint_priors():
    sigma = 45.0/ 180.0 * math.pi
    min_angle = 30 / 180.0 * math.pi
    reference_view_distance = 0.5
    gsd_view_distance = 0.5

    # generate 2 2d grids for the x & y bounds
    y, x = np.meshgrid(np.linspace(0.0, 1, 100), np.linspace(-0.5, 0.5, 100))

    dist = np.sqrt(x**2 + y**2)

    relative_area = (reference_view_distance / dist)**2
    resolution_prior = np.minimum(relative_area, 1/relative_area)
    relative_gsd = (gsd_view_distance / dist)**2
    gsd_prior = np.minimum(relative_gsd, 1/relative_gsd)

    z1 = np.multiply(resolution_prior, gsd_prior)

    theta = np.abs(np.arctan(x/y))
    trian_prior = 1 - (np.minimum(theta, min_angle) - min_angle)**2 / min_angle **2 # Triangulation Prior
    inc_prior = np.exp(-theta**2 / sigma**2)

    z2 = np.multiply(trian_prior, inc_prior)

    z = np.multiply(z1, z2)

    # x and y are bounds, so z should be the value *inside* those bounds.
    # Therefore, remove the last value from the z array.
    z = z[:-1, :-1]
    z_min, z_max = 0.0, 1.0

    fig, ax = plt.subplots()

    c = ax.pcolormesh(x, y, z, cmap='Blues', vmin=z_min, vmax=z_max)
    ax.set_title('Joint Geometric Prior')
    # set the limits of the plot to the limits of the data
    ax.axis([x.min(), x.max(), y.min(), y.max()])
    ax.set_aspect('equal', 'box')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('z [m]')
    fig.colorbar(c, ax=ax)


plot_range_deg=[-90, 90]
angle_deg = np.linspace(plot_range_deg[0], plot_range_deg[1], num=(plot_range_deg[1] - plot_range_deg[0] + 1))
angle = angle_deg * math.pi / 180.0

triangulation_prior = np.zeros(angle_deg.shape[0])
incident_prior = np.zeros(angle_deg.shape[0])

sigma = 45.0/ 180.0 * math.pi
min_angle = 30 / 180.0 * math.pi
ref_angle = 30/ 180.0 * math.pi

# region interpolation using a symmetric sigmoid function
# 0 in linear/quadratic region, 1 in post-stall region
for i in range(angle_deg.shape[0]):
    incident_prior[i] = math.exp(-angle[i]**2 / sigma**2)
    triangulation_angle = wrap_pi(abs(angle[i] - ref_angle))
    # triangulation_angle = angle[i] - ref_angle
    triangulation_prior[i] = 1 - (min(triangulation_angle, min_angle) - min_angle)**2 / min_angle **2

fig1, (ax11, ax21) = plt.subplots(2)

ax11.plot(angle_deg, triangulation_prior, label="triangulation prior")
ax11.plot(angle_deg, incident_prior, label="incident prior")
ax11.plot(angle_deg, incident_prior *incident_prior* triangulation_prior, label="joint prior")
ax11.set_title("Geometric Prior")
ax11.set_xlabel('Angle')
ax11.set_ylabel('Angle Priors')
ax11.legend()

distance = np.linspace(0.1, 100.0, num=100)

reference_view_distance = 50
gsd_view_distance = 100

gsd_prior = np.zeros(distance.shape[0])
resolution_prior = np.zeros(distance.shape[0])

for i in range(distance.shape[0]):
    relative_area = (reference_view_distance / distance[i])**2
    resolution_prior[i] = min(relative_area, 1/relative_area)
    relative_gsd = (gsd_view_distance / distance[i])**2
    gsd_prior[i] = min(relative_gsd, 1/relative_gsd)

ax21.plot(distance, resolution_prior, label="resolution prior")
ax21.plot(distance, gsd_prior, label="resolution prior")
ax21.plot(distance, resolution_prior*gsd_prior, label="joint prior")
ax21.set_xlabel('Distance')
ax21.set_ylabel('Distance Priors')
ax21.legend()

visualize_angular_priors()
plt.savefig('angular_prior.pgf')

visualize_distance_priors()

visualize_joint_priors()

plt.show()
plt.legend()
