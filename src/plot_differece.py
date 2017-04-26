#!/usr/bin/env python3
#-------------------------------------------------------------------------------
# Author: Lukasz Janyst <lukasz@jany.st>
# Date:   26.04.2017
#-------------------------------------------------------------------------------

import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import matplotlib.gridspec as pltgs

import sys

def plot(ax, ukf, ekf, idxs):
    ekf_data = []
    ukf_data = []
    for i in range(len(ukf)):
        u = ukf[i].split(' ')
        e = ekf[i].split(' ')
        ts = float(e[1])/1000000.
        gt = float(e[idxs[0]])
        ed = float(e[idxs[1]])
        ud = float(u[idxs[1]])
        ekf_data.append((ts, abs(ed-gt)))
        ukf_data.append((ts, abs(ud-gt)))
    edx, edy  = zip(*ekf_data)
    udx, udy  = zip(*ukf_data)
    ax.plot(edx, edy, 'b-', label="ekf")
    ax.plot(udx, udy, 'r-', label="ukf")
    ax.tick_params(which='both', bottom='off', top='off', labelbottom='off')

with open(sys.argv[1], "r") as f:
    dataUKF = f.readlines()

with open(sys.argv[2], "r") as f:
    dataEKF = f.readlines()

fig = plt.figure()
gs = pltgs.GridSpec(4, 1, height_ratios=([1, 1, 1, 1]))
ax_px = fig.add_subplot(gs[0])
ax_py = fig.add_subplot(gs[1])
ax_vx = fig.add_subplot(gs[2])
ax_vy = fig.add_subplot(gs[3])

ax_px.set_title('position x (m)')
ax_py.set_title('position y (m)')
ax_vx.set_title('velocity x (m/s)')
ax_vy.set_title('velocity y (m/s)')

plot(ax_px, dataUKF, dataEKF, [ 9, 4])
plot(ax_py, dataUKF, dataEKF, [10, 5])
plot(ax_vx, dataUKF, dataEKF, [11, 6])
plot(ax_vy, dataUKF, dataEKF, [12, 7])
legend = ax_px.legend(loc='upper right')
fig.set_size_inches(7, 7)
fig.savefig(sys.argv[3], bbox_inches='tight')
