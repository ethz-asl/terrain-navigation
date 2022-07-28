import numpy as np
import matplotlib.pyplot as plt

alpha = 0.3173325912716963 # rad
beta = 2.53866073017357 # rad
d = 2.30538

# center calculation
c_start_1 = (np.cos(alpha+0.5*np.pi), np.sin(alpha+0.5*np.pi))
c_start_2 = (np.cos(alpha-0.5*np.pi), np.sin(alpha-0.5*np.pi))
c_goal_1 = (d + np.cos(beta+0.5*np.pi), np.sin(beta+0.5*np.pi))
c_goal_2 = (d + np.cos(beta-0.5*np.pi), np.sin(beta-0.5*np.pi))

# draw circles
fig, ax = plt.subplots(1, 1)
circle_s1 = plt.Circle(c_start_1, 1, color='b', fill=False)
circle_s2 = plt.Circle(c_start_2, 1, color='b', fill=False)
circle_g1 = plt.Circle(c_goal_1, 1, color='b', fill=False)
circle_g2 = plt.Circle(c_goal_2, 1, color='b', fill=False)
ax.add_patch(circle_s1)
ax.add_patch(circle_s2)
ax.add_patch(circle_g1)
ax.add_patch(circle_g2)
ax.axis('equal')
ax.set_xlim(-1,d+1)
ax.arrow(0, 0, 0.5*np.cos(alpha), 0.5*np.sin(alpha), head_width=0.05, head_length=0.1, fc='k', ec='k')
ax.arrow(d, 0, 0.5*np.cos(beta), 0.5*np.sin(beta), head_width=0.05, head_length=0.1, fc='k', ec='k')
plt.show()