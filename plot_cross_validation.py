import numpy as np

import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

from matplotlib import rc
rc('font',**{'family':'serif','serif':['Times']})
rc('text', usetex=True)

import matplotlib
from mpl_toolkits.axes_grid1 import make_axes_locatable
import matplotlib.pyplot as plt
import cmocean
import math

data = np.zeros((19, 19))
def read_cross_val(file):
    cvmin = float("inf")
    cross_val = []
    with open(file, "r") as f:
        f.readline()
        for line in f:
            line = line.split(",")
            test = line[0]
            baseline = line[-1]
            norms = np.array(line[2:-1:2]).astype(float)
            cross_val += [norms]
            cvmin = min(cvmin, np.min(norms))
    print(cvmin)
    return cross_val

#data = [np.array(read_cross_val("data/mar27_1426_circ_audio/cross_val_1.csv"))]
data = [np.transpose(np.array([
    np.mean(
        [np.array([0.8795830594, 0.2954323802, 0.6221439759, 0.2042102136]),
        np.array([0.8918489999, 0.4567163593, 0.5342592709, 0.2185999738])],
        axis=0),
    np.mean(
        [np.array([0.915340749, 0.4098302396, 0.6136989772, 0.205107859]),
        np.array([0.7225828716, 0.4366292816, 0.5954626958, 0.2067444847])],
        axis=0),
    np.mean(
        [np.array([0.8655534877, 0.3725220808, 0.6447098379, 0.2116129483]),
        np.array([0.7422649587, 0.332037219, 0.5376501762, 0.1965236993])],
        axis=0),
    np.mean(
        [np.array([1.000929665, 0.4229830169, 0.5094500225, 0.2146300293]),
        np.array([0.9179843178, 0.4466214787, 0.6280712363, 0.1956162684])],
        axis=0),
    np.mean(
        [np.array([1.132310332, 0.431649494, 0.4934880622, 0.2005070989]),
        np.array([0.8090878785, 0.3011411059, 0.5359151805, 0.202457734])],
        axis=0)
]))]

fig = plt.subplots()

# normal cmap
ax = plt.gca()
im0 = ax.imshow(
    data[0],
    interpolation="none",
    cmap="RdBu_r",
    norm=mcolors.LogNorm(vmax=10, vmin=0.1),
)
divider = make_axes_locatable(ax)
cax = divider.append_axes("right", size="5%", pad=0.05)
cbar = plt.colorbar(im0, ticks=[0.1, 0.2, 0.5, 1, 2, 5, 10], cax=cax)
cbar.ax.set_yticklabels(
    [
        ("%sx") % (("%f" % i).rstrip("0").rstrip("."))
        for i in [0.1, 0.2, 0.5, 1, 2, 5, 10]
    ], fontsize=27
)
ax.set_xticks(np.arange(0, 5, 1))
#ax.set_xticklabels([r"Clamp",r"Clamp$_{0..1}$",r"tanh",r"tanh$_{0..1}$",r"ISRU",r"ISRU$_{0..1}$",r"atan",r"atan$_{0..1}$", r"Softsign", r"\hspace{30pt}Softsign$_{0..1}$"], fontsize=27)
ax.set_xticklabels([r"Clamp",r"tanh",r"ISRU",r"atan", r"Softsign"], fontsize=27)
ax.set_xlabel("Communication Non-Linearity", fontsize=32)
ax.set_yticks(np.arange(0, 4, 1))
ax.set_yticklabels(["A","C","D","F"],fontsize=27)
ax.set_ylabel("Testing Scenario",fontsize=32)

plt.subplots_adjust(left=0.1, right=0.9, top=1.0, bottom=0)
plt.show()
