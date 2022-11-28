import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator)

import math

def mod2pi(x):
    return np.mod(x, 2 * np.pi)


numel = 100
beta, alpha = np.meshgrid(np.linspace(0.0, 2*math.pi, numel), np.linspace(0.0, 2*math.pi, numel))

ca = np.cos(alpha)
cb = np.cos(beta)
sa = np.sin(alpha)
sb = np.sin(beta)

d = 2.30538

long_path_threshold = d - (np.sqrt(4.0 - (np.abs(ca) + np.abs(cb))**2) + np.abs(sa) + np.abs(sb))

p_lsr = (d * d) - 2.0 + 2.0 * (ca * cb + sa * sb + d * (sa + sb))

mask_longpath = long_path_threshold > 0
mask_plsr =  p_lsr < 0

violated = np.logical_and(mask_plsr, mask_longpath)
fig, ax = plt.subplots(1, 1)
c = ax.pcolormesh(alpha, beta, violated, cmap='bwr', vmin=0, vmax=1)
ax.set_title("violated")
ax.set_xlabel('alpha')
ax.set_ylabel('beta')
ax.xaxis.set_major_locator(MultipleLocator(0.5* math.pi))
ax.yaxis.set_major_locator(MultipleLocator(0.5* math.pi))
ax.grid(True)
fig.colorbar(c, ax=ax)

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



# brute force path length computation
# LSL
p_lsl_squared = 2 + d**2 - 2 * (ca * cb + sa * sb - d * (sa - sb))
p_lsl_squared[p_lsl_squared < 0] = np.inf
p_lsl = np.sqrt(p_lsl_squared)
lsl_atan = np.arctan2(cb-ca, d+sa-sb)
t_lsl = mod2pi(-alpha + lsl_atan)
q_lsl = mod2pi(mod2pi(beta) - lsl_atan)
L_lsl = t_lsl + p_lsl + q_lsl

# RSR
p_rsr_squared = 2 + d**2 - 2 * (ca * cb + sa * sb - d * (sb - sa))
p_rsr_squared[p_rsr_squared < 0] = np.inf
p_rsr = np.sqrt(p_rsr_squared)
rsr_atan = np.arctan2(ca-cb, d-sa+sb)
t_rsr = mod2pi(alpha - rsr_atan)
q_rsr = mod2pi(-mod2pi(beta) + rsr_atan)
L_rsr = t_rsr + p_rsr + q_rsr

# LSR
p_lsr_squared = -2 + d**2 + 2 * (ca * cb + sa * sb + d * (sa + sb))
p_lsr_squared[p_lsr_squared < 0] = np.inf
p_lsr = np.sqrt(p_lsr_squared)
lsr_atan = np.arctan2(-ca-cb, d+sa+sb) - np.arctan2(-2.0, p_lsr)
t_lsr = mod2pi(-alpha + lsr_atan)
q_lsr = mod2pi(-mod2pi(beta) + lsr_atan)
L_lsr = t_lsr + p_lsr + q_lsr

# RSL
p_rsl_squared = -2 + d**2 + 2 * (ca * cb + sa * sb - d * (sa + sb))
p_rsl_squared[p_rsl_squared < 0] = np.inf
p_rsl = np.sqrt(p_rsl_squared)
rsl_atan = np.arctan2(ca+cb, d-sa-sb) - np.arctan2(2.0, p_rsl)
t_rsl = mod2pi(alpha - rsl_atan)
q_rsl = mod2pi(mod2pi(beta) - rsl_atan)
L_rsl = t_rsl + p_rsl + q_rsl

# LRL
p_lrl = 0.125 * (6 - d**2 + 2*(ca * cb + sa * sb - d*(sa -sb)))
p_lrl[np.abs(p_lrl) <= 1] = np.arccos(p_lrl[np.abs(p_lrl) <= 1])
p_lrl[np.abs(p_lrl) > 1] = np.inf
p_lrl = mod2pi(p_lrl)
t_lrl = mod2pi(-alpha + np.arctan2(-ca+cb, d+sa-sb) + 0.5*p_lrl)
q_lrl = mod2pi(-alpha + mod2pi(beta) - t_lrl + p_lrl)
L_lrl = t_lrl + p_lrl + q_lrl

# RLR
p_rlr = 0.125 * (6 - d**2 + 2*(ca * cb + sa * sb + d*(sa -sb)))
p_rlr[np.abs(p_rlr) <= 1] = np.arccos(p_rlr[np.abs(p_rlr) <= 1])
p_rlr[np.abs(p_rlr) > 1] = np.inf
p_rlr = mod2pi(p_rlr)
t_rlr = mod2pi(alpha - np.arctan2(ca-cb, d-sa+sb) + 0.5*p_rlr)
q_rlr = mod2pi(alpha - mod2pi(beta) - t_rlr + p_rlr)
L_rlr = t_rlr + p_rlr + q_rlr


stacked_length = np.stack([L_lsl, L_rsr, L_lsr, L_rsl, L_lrl, L_rlr])
shortest_path = np.nanargmin(stacked_length, axis=0)

def getShortestPathClassification(long_path_threshold):
    mask_longpath = long_path_threshold > 0

    # brute force path length computation
    # LSL
    p_lsl_squared = 2 + d**2 - 2 * (ca * cb + sa * sb - d * (sa - sb))
    p_lsl_squared[p_lsl_squared < 0] = np.inf
    p_lsl = np.sqrt(p_lsl_squared)
    lsl_atan = np.arctan2(cb-ca, d+sa-sb)
    t_lsl = mod2pi(-alpha + lsl_atan)
    q_lsl = mod2pi(mod2pi(beta) - lsl_atan)
    L_lsl = t_lsl + p_lsl + q_lsl

    # RSR
    p_rsr_squared = 2 + d**2 - 2 * (ca * cb + sa * sb - d * (sb - sa))
    p_rsr_squared[p_rsr_squared < 0] = np.inf
    p_rsr = np.sqrt(p_rsr_squared)
    rsr_atan = np.arctan2(ca-cb, d-sa+sb)
    t_rsr = mod2pi(alpha - rsr_atan)
    q_rsr = mod2pi(-mod2pi(beta) + rsr_atan)
    L_rsr = t_rsr + p_rsr + q_rsr

    # LSR
    p_lsr_squared = -2 + d**2 + 2 * (ca * cb + sa * sb + d * (sa + sb))
    p_lsr_squared[p_lsr_squared < 0] = np.inf
    p_lsr = np.sqrt(p_lsr_squared)
    lsr_atan = np.arctan2(-ca-cb, d+sa+sb) - np.arctan2(-2.0, p_lsr)
    t_lsr = mod2pi(-alpha + lsr_atan)
    q_lsr = mod2pi(-mod2pi(beta) + lsr_atan)
    L_lsr = t_lsr + p_lsr + q_lsr

    # RSL
    p_rsl_squared = -2 + d**2 + 2 * (ca * cb + sa * sb - d * (sa + sb))
    p_rsl_squared[p_rsl_squared < 0] = np.inf
    p_rsl = np.sqrt(p_rsl_squared)
    rsl_atan = np.arctan2(ca+cb, d-sa-sb) - np.arctan2(2.0, p_rsl)
    t_rsl = mod2pi(alpha - rsl_atan)
    q_rsl = mod2pi(mod2pi(beta) - rsl_atan)
    L_rsl = t_rsl + p_rsl + q_rsl

    # LRL
    p_lrl = 0.125 * (6 - d**2 + 2*(ca * cb + sa * sb - d*(sa -sb)))
    p_lrl[np.abs(p_lrl) <= 1] = np.arccos(p_lrl[np.abs(p_lrl) <= 1])
    p_lrl[np.abs(p_lrl) > 1] = np.inf
    p_lrl = mod2pi(p_lrl)
    t_lrl = mod2pi(-alpha + np.arctan2(-ca+cb, d+sa-sb) + 0.5*p_lrl)
    q_lrl = mod2pi(-alpha + mod2pi(beta) - t_lrl + p_lrl)
    L_lrl = t_lrl + p_lrl + q_lrl

    # RLR
    p_rlr = 0.125 * (6 - d**2 + 2*(ca * cb + sa * sb + d*(sa -sb)))
    p_rlr[np.abs(p_rlr) <= 1] = np.arccos(p_rlr[np.abs(p_rlr) <= 1])
    p_rlr[np.abs(p_rlr) > 1] = np.inf
    p_rlr = mod2pi(p_rlr)
    t_rlr = mod2pi(alpha - np.arctan2(ca-cb, d-sa+sb) + 0.5*p_rlr)
    q_rlr = mod2pi(alpha - mod2pi(beta) - t_rlr + p_rlr)
    L_rlr = t_rlr + p_rlr + q_rlr

    S12 = p_rsr - p_rsl - 2.0 * (q_rsl - np.pi)
    S13 = t_rsr - np.pi
    S14_1 = t_rsr - np.pi
    S14_2 = q_rsr - np.pi
    S21 = p_lsl - p_rsl - 2.0*(t_rsl - np.pi)
    S22_1 = p_lsl - p_rsl - 2.0*(t_rsl - np.pi)
    S22_2 = p_rsr - p_rsl - 2.0*(q_rsl - np.pi)
    S24 = q_rsr - np.pi
    S31 = q_lsl - np.pi
    S33_1 = p_rsr - p_lsr - 2.0*(t_lsr - np.pi)
    S33_2 = p_lsl - p_lsr - 2.0*(q_lsr - np.pi)
    S34 = p_rsr - p_lsr - 2.0*(t_lsr - np.pi)
    S41_1 = t_lsl - np.pi
    S41_2 = q_lsl - np.pi
    S42 = t_lsl - np.pi
    S43 = p_lsl - p_lsr - 2.0*(q_lsr - np.pi)

    shortest_path_classification = np.ones_like(shortest_path) * -1
    for i in range(numel):
        for j in range(numel):
            if mask_longpath[i, j]:
                if ((alpha[i, j] <= 0.5 * np.pi) and
                    (beta[i, j] <= 0.5 * np.pi)):
                    shortest_path_classification[i, j] = 3
                    
                elif ((alpha[i, j] <= 0.5 * np.pi) and
                    (beta[i, j] > 0.5 * np.pi) and
                    (beta[i, j] <= np.pi)):
                    if S12[i, j] > 0:
                        shortest_path_classification[i, j] = 3
                    else:
                        shortest_path_classification[i, j] = 1

                elif ((alpha[i, j] <= 0.5 * np.pi) and
                    (beta[i, j] > np.pi) and
                    (beta[i, j] <= 1.5 * np.pi)):
                    if S13[i, j] > 0:
                        shortest_path_classification[i, j] = 2
                    else:
                        shortest_path_classification[i, j] = 1

                elif ((alpha[i, j] <= 0.5 * np.pi) and
                    (beta[i, j] > 1.5 * np.pi) and
                    (beta[i, j] <= 2.0 * np.pi)):
                    if S14_1[i, j] > 0:
                        shortest_path_classification[i, j] = 2
                    elif S14_2[i, j] > 0:
                        shortest_path_classification[i, j] = 3
                    else:
                        shortest_path_classification[i, j] = 1

                elif ((alpha[i, j] > 0.5 * np.pi) and
                    (alpha[i, j] <= np.pi) and
                    (beta[i, j] <= 0.5 * np.pi)):
                    if S21[i, j] > 0:
                        shortest_path_classification[i, j] = 3
                    else:
                        shortest_path_classification[i, j] = 0

                elif ((alpha[i, j] > 0.5 * np.pi) and
                    (alpha[i, j] <= np.pi) and
                    (beta[i, j] > 0.5 * np.pi) and
                    (beta[i, j] <= np.pi)):
                    if alpha[i, j] > beta[i, j]:
                        if S22_1[i, j] > 0:
                            shortest_path_classification[i, j] = 3
                        else:
                            shortest_path_classification[i, j] = 0
                    else:
                        if S22_2[i, j] > 0:
                            shortest_path_classification[i, j] = 3
                        else:
                            shortest_path_classification[i, j] = 1

                elif ((alpha[i, j] > 0.5 * np.pi) and
                    (alpha[i, j] <= np.pi) and
                    (beta[i, j] > np.pi) and
                    (beta[i, j] <= 1.5 * np.pi)):
                    shortest_path_classification[i, j] = 1

                elif ((alpha[i, j] > 0.5 * np.pi) and
                    (alpha[i, j] <= np.pi) and
                    (beta[i, j] > 1.5 * np.pi) and
                    (beta[i, j] <= 2.0 * np.pi)):
                    if S24[i, j] > 0:
                        shortest_path_classification[i, j] = 3
                    else:
                        shortest_path_classification[i, j] = 1

                elif ((alpha[i, j] > np.pi) and
                    (alpha[i, j] <= 1.5 * np.pi) and
                    (beta[i, j] <= 0.5 * np.pi)):
                    if S31[i, j] > 0:
                        shortest_path_classification[i, j] = 2
                    else:
                        shortest_path_classification[i, j] = 0

                elif ((alpha[i, j] > np.pi) and
                    (alpha[i, j] <= 1.5 * np.pi) and
                    (beta[i, j] > 0.5 * np.pi) and
                    (beta[i, j] <= np.pi)):
                    shortest_path_classification[i, j] = 0

                elif ((alpha[i, j] > np.pi) and
                    (alpha[i, j] <= 1.5 * np.pi) and
                    (beta[i, j] > np.pi) and
                    (beta[i, j] <= 1.5 * np.pi)):
                    if alpha[i, j] > beta[i, j]:
                        if S33_2[i, j] > 0:
                            shortest_path_classification[i, j] = 2
                        else:
                            shortest_path_classification[i, j] = 0
                    else:
                        if S33_1[i, j] > 0:
                            shortest_path_classification[i, j] = 2
                        else:
                            shortest_path_classification[i, j] = 1

                elif ((alpha[i, j] > np.pi) and
                    (alpha[i, j] <= 1.5 * np.pi) and
                    (beta[i, j] > 1.5 * np.pi) and
                    (beta[i, j] <= 2.0 * np.pi)):
                    if S34[i, j] > 0:
                        shortest_path_classification[i, j] = 2
                    else:
                        shortest_path_classification[i, j] = 1

                elif ((alpha[i, j] > 1.5 * np.pi) and
                    (alpha[i, j] <= 2.0 * np.pi) and
                    (beta[i, j] <= 0.5 * np.pi)):
                    if S41_1[i, j] > 0:
                        shortest_path_classification[i, j] = 3
                    elif S41_2[i, j] > 0:
                        shortest_path_classification[i, j] = 2
                    else:
                        shortest_path_classification[i, j] = 0

                elif ((alpha[i, j] > 1.5 * np.pi) and
                    (alpha[i, j] <= 2.0 * np.pi) and
                    (beta[i, j] > 0.5 * np.pi) and
                    (beta[i, j] <= np.pi)):
                    if S42[i, j] > 0:
                        shortest_path_classification[i, j] = 3
                    else:
                        shortest_path_classification[i, j] = 0

                elif ((alpha[i, j] > 1.5 * np.pi) and
                    (alpha[i, j] <= 2.0 * np.pi) and
                    (beta[i, j] > np.pi) and
                    (beta[i, j] <= 1.5 * np.pi)):
                    if S43[i, j] > 0 and np.isreal(p_lsr[i, j]):
                        shortest_path_classification[i, j] = 2
                    else:
                        shortest_path_classification[i, j] = 0

                elif ((alpha[i, j] > 1.5 * np.pi) and
                    (alpha[i, j] <= 2.0 * np.pi) and
                    (beta[i, j] > 1.5 * np.pi) and
                    (beta[i, j] <= 2.0 * np.pi)):
                    shortest_path_classification[i, j] = 2

            else:
                shortest_path_classification[i, j] = shortest_path[i, j]
    return shortest_path_classification


fig, ax = plt.subplots(1, 3)
c = ax[0].pcolormesh(alpha, beta, shortest_path, cmap='jet', vmin=0, vmax=5)
ax[0].set_title("Exhaustive")
ax[0].set_xlabel('alpha')
ax[0].set_ylabel('beta')
ax[0].xaxis.set_major_locator(MultipleLocator(0.5* math.pi))
ax[0].yaxis.set_major_locator(MultipleLocator(0.5* math.pi))
ax[0].grid(True)
# fig.colorbar(c, ax=ax[0])

long_path_threshold = d - (np.sqrt(4.0 - (np.abs(ca) + np.abs(cb))**2) + np.abs(sa) + np.abs(sb))

shortest_path_classification = getShortestPathClassification(long_path_threshold)

c = ax[1].pcolormesh(alpha, beta, shortest_path_classification, cmap='jet', vmin=0, vmax=5)
ax[1].set_title("Dubins Set Classification")
ax[1].set_xlabel('alpha')
# ax[1].set_ylabel('beta')
ax[1].xaxis.set_major_locator(MultipleLocator(0.5* math.pi))
ax[1].yaxis.set_major_locator(MultipleLocator(0.5* math.pi))
ax[1].grid(True)
# fig.colorbar(c, ax=ax[1])

long_path_threshold = d - (np.sqrt(4.0 - (ca + cb)**2) + np.abs(sa) + np.abs(sb))

shortest_path_classification = getShortestPathClassification(long_path_threshold)
c = ax[2].pcolormesh(alpha, beta, shortest_path_classification, cmap='jet', vmin=0, vmax=5)
ax[2].set_title("Proposed")
ax[2].set_xlabel('alpha')
# ax[2].set_ylabel('beta')
ax[2].xaxis.set_major_locator(MultipleLocator(0.5* math.pi))
ax[2].yaxis.set_major_locator(MultipleLocator(0.5* math.pi))
ax[2].grid(True)
# fig.colorbar(c, ax=ax[2])

fig, ax = plt.subplots(1, 1)
c = ax.pcolormesh(alpha, beta, shortest_path_classification - shortest_path, cmap='bwr')
ax.set_title("classification error")
ax.set_xlabel('alpha')
ax.set_ylabel('beta')
ax.xaxis.set_major_locator(MultipleLocator(0.5* math.pi))
ax.yaxis.set_major_locator(MultipleLocator(0.5* math.pi))
ax.grid(True)
fig.colorbar(c, ax=ax)

plt.tight_layout()
plt.show()
