#!/usr/bin/env python3
# This python script visualizes the geometric priors
import matplotlib.pyplot as plt
import numpy as np
import math
import pandas as pd
import sys


def getFIM(view_vector) :
    distance = np.linalg.norm(view_vector)
    Jacobian =  1/distance * np.identity(3) - (1/np.power(distance, 3)) * np.outer(view_vector, view_vector)
    # HandJacobian = (1/np.power(distance, 3)) * np.array([[distance**2 - view_vector[0]**2, -view_vector[0] *view_vector[1], -view_vector[0] *view_vector[2]],
    #                          [-view_vector[0] *view_vector[1], distance**2 - view_vector[1]**2, -view_vector[1] *view_vector[2]],
    #                          [-view_vector[0] *view_vector[2], -view_vector[1] *view_vector[2], distance**2 - view_vector[2]**2]])
    pixel_res = 0.5 * math.pi / 1080.0 # Angle resolution of a single pixel
    sigma = math.sin(pixel_res) # Angle resolution of a single pixel
    FIM = (1/np.power(sigma, 2)) * np.matmul(np.transpose(Jacobian), Jacobian)
    return FIM

def getEopt(FIM) :
    w, v = np.linalg.eig(FIM)
    return np.min(w)

def getDopt(FIM) :
    return np.linalg.det(FIM)

accumulated_fim = np.zeros((3,3))
information_e = np.array([0.0])
information_d = np.array([0.0])
bound_x = np.array([0.0])
bound_y = np.array([0.0])
bound_z = np.array([0.0])

for i in range(20):
    rand_bearing = np.random.rand(3)
    rand_bearing = 100.0 * rand_bearing / np.linalg.norm(rand_bearing)
    fim = getFIM(rand_bearing)
    accumulated_fim = np.add(accumulated_fim, fim)
    eopt = getEopt(accumulated_fim)
    if eopt > 0 :
        information_e = np.append(information_e, eopt)
        cramer_rao_bound = np.sqrt(np.diag(np.linalg.inv(accumulated_fim)))
        bound_x = np.append(bound_x, cramer_rao_bound[0])
        bound_y = np.append(bound_y, cramer_rao_bound[1])
        bound_z = np.append(bound_z, cramer_rao_bound[2])

    dopt = getDopt(accumulated_fim)
    information_d = np.append(information_d, dopt)

fig = plt.figure("Information")
# ax1 = fig.add_subplot(1, 3, 1)
# ax1.plot(information_e)
# ax1.set_title("E-Optimality")
# ax1.set_xlabel("Number of views")

# ax2 = fig.add_subplot(1, 3, 2)
# ax2.plot(information_d)
# ax2.set_title("D-Optimality")
# ax2.set_xlabel("Number of views")

ax3 = fig.add_subplot(1, 1, 1)
ax3.plot(bound_x[bound_x !=0], '-o')
ax3.plot(bound_y[bound_y !=0], '-o')
ax3.plot(bound_z[bound_z !=0], '-o')
ax3.set_title("Cramer Rao Bounds [m]")
ax3.set_xlabel("Number of views")
ax3.set_ylabel("Cramer Rao Bounds [m]")
ax3.legend(['X [m]', 'Y [m]', 'Z [m]'])
ax3.set_xticks(range(0, 20, 5))
ax3.grid()

plt.show()
