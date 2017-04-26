#!/usr/bin/env python3
#-------------------------------------------------------------------------------
# Author: Lukasz Janyst <lukasz@jany.st>
# Date:   26.04.2017
#-------------------------------------------------------------------------------

import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import sys

lidarPos  = []
radarPos  = []
kalmanPos = []
lidarNis  = []
radarNis  = []

with open(sys.argv[1], "r") as f:
    i1 = 0
    i2 = 0
    for l in f.readlines():
        l = l.split(' ')
        kalmanPos.append((float(l[4]), float(l[5])))
        if l[0] == 'L':
            lidarPos.append((float(l[2]), float(l[3])))
            lidarNis.append((i1, float(l[8])))
            i1 += 1
        else:
            radarPos.append((float(l[2]), float(l[3])))
            radarNis.append((i2, float(l[8])))
            i2 += 1

prefix = sys.argv[2]

lx,  ly  = zip(*lidarPos)
rx,  ry  = zip(*radarPos)
kx,  ky  = zip(*kalmanPos)
nlx, nly = zip(*lidarNis)
nrx, nry = zip(*radarNis)

fig, ax = plt.subplots()
ax.plot(lx, ly, 'y.', label="lidar data")
ax.plot(rx, ry, 'b.', label="radar data")
ax.plot(kx, ky, 'r-', label="estimated trajectory")
legend = ax.legend(loc='upper left')
ax.set_title(prefix + " position")
fig.set_size_inches(7, 7)
fig.savefig(prefix+'_position.png', bbox_inches='tight')

fig, ax = plt.subplots()
fig.set_size_inches(7, 7)
ax.plot(nrx, nry, 'r-', label="radar NIS")
ax.axhline(y=7.815, xmin=0, xmax=i1, c="blue", label="chi^2 - 3 df - 0.05")
legend = ax.legend(loc='upper left')
ax.set_title(prefix + " radar NIS")
fig.savefig(prefix+'_nis_radar.png', bbox_inches='tight')

fig, ax = plt.subplots()
fig.set_size_inches(7, 7)
ax.plot(nlx, nly, 'r-', label="lidar NIS")
ax.axhline(y=5.991, xmin=0, xmax=i2, c="blue", label="chi^2 - 2 df - 0.05")
legend = ax.legend(loc='upper left')
ax.set_title(prefix + " lidar NIS")
fig.savefig(prefix+'_nis_lidar.png', bbox_inches='tight')
