import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib import cm

class MplColorHelper:

  def __init__(self, cmap_name, start_val, stop_val):
    self.cmap_name = cmap_name
    self.cmap = plt.get_cmap(cmap_name)
    self.norm = mpl.colors.Normalize(vmin=start_val, vmax=stop_val)
    self.scalarMap = cm.ScalarMappable(norm=self.norm, cmap=self.cmap)

  def get_rgb(self, val):
    return self.scalarMap.to_rgba(val)

import numpy as np
# setup the plot
fig, ax = plt.subplots(1,1, figsize=(6,6))

# define the data between 0 and 20
NUM_VALS = 20
# x = np.random.uniform(0, NUM_VALS, size=NUM_VALS)
# y = np.random.uniform(0, NUM_VALS, size=NUM_VALS)

# define the color chart between 2 and 10 using the 'autumn_r' colormap, so
#   y <= 2  is yellow
#   y >= 10 is red
#   2 < y < 10 is between from yellow to red, according to its value

colormap_name = 'winter'
COL = MplColorHelper(colormap_name, 0.0, 1.0)

f= open('colormap.txt', 'w')
# print(intensity)
f.write('{\"'+colormap_name+'\", {')

step = 1.0/255.0
intensity = np.arange(0.0, 1.0 + step, step)
for y in intensity:
    v = COL.get_rgb(y)
    f.write('{' + str(v[0]) +  ', ' + str(v[1]) + ', ' + str(v[2]) + '}')

    if y==1.0:
        f.write('}')
    else:
        f.write(', ')

f.close()

# scat = ax.scatter(x,y,s=300, c=COL.get_rgb(y))
# ax.set_title('Well defined discrete colors')
# plt.show()