import csv
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.ticker import (MultipleLocator, AutoMinorLocator)

matplotlib.use('TKAgg')

df = pd.read_csv("/home/marcos/OneDrive/home/ETSIT/tercero/irin/o1/dev/irsim2021/outputFiles/robotPosition")

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

plt.scatter(rdatax, rdatay, cmap="plasma", s=0.75)

cbar = plt.colorbar()
print(tdata)
plt.clim(tdata[0], tdata[-1])
cbar.ax.set_ylabel('m_fTime')

plt.scatter(0.6, 0.6, s=40, c="yellow", edgecolors="black")

plt.show()
