import csv
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import scipy.ndimage as sp
from matplotlib.ticker import (MultipleLocator, AutoMinorLocator)

matplotlib.use('TKAgg')

pos = []

with open("/home/marcos/OneDrive/home/ETSIT/tercero/irin/o1/dev/irsim2021/outputFiles/robotPosition", "r") as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=' ')
    line_count = 0
    for row in csv_reader:
            pos.append((row[1], row[2]))
            line_count += 1
    print(f'Processed {line_count} lines.')

pos = np.array(pos, dtype=np.float64)

rdatax, rdatay = pos.T

fig = plt.figure()
axes = fig.gca()

axes.set_aspect('equal', adjustable='box')

axes.xaxis.set_minor_locator(AutoMinorLocator())
axes.yaxis.set_minor_locator(AutoMinorLocator())

axes.set_axisbelow(True)

plt.grid(linestyle='--', linewidth=0.5, which="major")
plt.grid(linestyle='-.', linewidth=0.25, which="minor")

fig.suptitle('Robot position during experiment 1')
plt.xlabel('x pos [m]')
plt.ylabel('y pos [m]')
rdatax = sp.filters.gaussian_filter(rdatax, sigma = 4, order = 0)
rdatay = sp.filters.gaussian_filter(rdatay, sigma = 4, order = 0)
plt.hexbin(rdatax, rdatay, bins=100, cmap="plasma")

cbar = plt.colorbar()
cbar.ax.set_ylabel('simulation step nr.')

plt.scatter(0.6, 0.6, s=40, c="yellow", edgecolors="black")

plt.show()