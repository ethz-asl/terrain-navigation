import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator)

import math


beta, alpha = np.meshgrid(np.linspace(0.0, 2*math.pi, 100), np.linspace(0.0, 2*math.pi, 100))

ca = np.cos(alpha)
cb = np.cos(beta)
sa = np.sin(alpha)
sb = np.sin(beta)

d = 2.30538

long_path_threshold = d - (np.sqrt(4.0 - (np.abs(ca) + np.abs(cb))**2) + np.abs(sa) + np.abs(sb))

p_lsr = (d * d) - 2.0 + 2.0 * (ca * cb + sa * sb - d * (sa + sb))

fig, ax = plt.subplots(1, 2)
c = ax[0].pcolormesh(alpha, beta, p_lsr, cmap='bwr', vmin=-2, vmax=2)
ax[0].set_title("p_rsl ^2")
ax[0].set_xlabel('alpha')
ax[0].set_ylabel('beta')
ax[0].xaxis.set_major_locator(MultipleLocator(0.5* math.pi))
ax[0].yaxis.set_major_locator(MultipleLocator(0.5* math.pi))
ax[0].grid(True)
fig.colorbar(c, ax=ax[0])

c2 = ax[1].pcolormesh(alpha, beta, long_path_threshold, cmap='bwr', vmin=-2, vmax=2)
ax[1].set_title("long_path_threshold")
ax[1].set_xlabel('alpha')
ax[1].set_ylabel('beta')
ax[1].xaxis.set_major_locator(MultipleLocator(0.5* math.pi))
ax[1].yaxis.set_major_locator(MultipleLocator(0.5* math.pi))
ax[1].grid(True)
fig.colorbar(c2, ax=ax[1])
plt.show()
