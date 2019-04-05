import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import numpy as np

import numpy as np
import matplotlib
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

data = [np.array(read_cross_val("data/mar27_1426_circ_audio/cross_val_1.csv"))]

fig = plt.subplots()

# normal cmap
im0 = plt.imshow(
    data[0],
    interpolation="none",
    cmap="RdBu_r",
    norm=mcolors.LogNorm(vmax=10, vmin=0.1),
)
cbar = plt.colorbar(im0, ticks=[0.1, 0.2, 0.5, 1, 2, 5, 10])
cbar.ax.set_yticklabels(
    [
        ("%sx") % (("%f" % i).rstrip("0").rstrip("."))
        for i in [0.1, 0.2, 0.5, 1, 2, 5, 10]
    ]
)
plt.xticks(np.arange(0, 8, 1), ["A","B","C","D","E","F","G","H"])#[2, 4, 8, 9, 12, 13, 15, 18])
plt.xlabel("Model")
plt.yticks(np.arange(0, 8, 1), ["A","B","C","D","E","F","G","H"])#[2, 4, 8, 9, 12, 13, 15, 18])
plt.ylabel("Test")

plt.subplots_adjust(left=0.0, right=1.0, top=0.90, bottom=0.10)
plt.show()
