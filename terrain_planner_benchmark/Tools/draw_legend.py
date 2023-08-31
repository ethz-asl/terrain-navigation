import numpy as np
from matplotlib import pyplot as plt
x = np.linspace(1, 100, 1000)
y = np.log(x)
y1 = np.sin(x)
fig = plt.figure("Line plot")
legendFig = plt.figure("Legend plot")
ax = fig.add_subplot(111)
line1, = ax.plot(x, y, c=(0.0, 1.0, 1.0), lw=4, linestyle="dashdot")
line2, = ax.plot(x, y1, c=(1.0, 0.0, 1.0), lw=4, linestyle="dashdot")
legendFig.legend([line1, line2], ["Proposed", "Position + Heading"], loc='center')

plt.tight_layout
plt.show()