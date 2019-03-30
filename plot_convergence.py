import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker

# 0 & 1: circle
# 2 & 3: circle with obstacles
# 4 & 5 & 6: intersection
# 7 & 8: simple door
# 9: symmetric door
# 10: asymmetric door
# 11: evacuation
# 12: crowd
# 13: hall
# 14 & 15 & 16 & 17: (asymmetric) simple
# 18: two doors

scn_colors = [
    "#D62728", "#FF9896", ### reds
    "#E377C2", "#F7B6D2", ### pinks
    "#E6550D", "#FD8D3C", "#FDAE6B", # orange
    "#637939", "#8CA252", ###################### forest
    "#BCBD22", ############################ d. yellow
    "#DBDB8D", ############################ yellow
    "#1F77B4", #### d. blue
    "#9E9AC8", ############## purple
    "#8C564B", ########################## brown
    "#31A354", "#74C476", "#A1D99B","#C7E9C0", # greens
    "#AEC7E8" ##### l. blue
]

# separate plots
fig, ax = plt.subplots(nrows=3, ncols=1)
fig.tight_layout()
ax[0] = plt.subplot2grid((3, 20), (0, 0), colspan=20)
ax[1] = plt.subplot2grid((3, 20), (1, 0), colspan=9)
ax[2] = plt.subplot2grid((3, 20), (2, 0), colspan=9)
for sub, kind in enumerate(["mar27_1426_circ_audio_noiters"]):
    # separate lines
    max_c = 0
    min_c = float('inf')
    for line in range(18 + 1):
        its = []
        cs = []
        with open("data/%s/t%d_train/convergence.log" % (kind, line), "r") as f:
            f.readline()  # skip header
            for point in f:
                it, c = point.split(",")
                its += [int(it.split(":")[1].strip())]
                cs += [float(c.split(":")[1].strip())]
        ax[sub].plot(its, cs, linewidth=1, color=scn_colors[line])
        max_c = np.max([max_c] + list(cs))
        min_c = np.min([min_c] + list(cs))
    ax[sub].margins(x=0)
    c10 = (max_c - min_c) / 8.
    cr = np.arange(min_c, max_c+c10, c10)
    ir = np.arange(its[0], its[-1]+25, 25)
    ax[sub].set_yticks(cr)
    if sub == 0:
        ax[sub].set_yticklabels(["%.2f" % i for i in cr])
    else: 
        ax[sub].set_yticklabels(["%.0f" % i for i in cr])
    ax[sub].set_xticks(ir)
    ax[sub].set_xticklabels(ir, rotation=90)
    ax[sub].set_title(kind.split("_")[-1].capitalize(), rotation=90, loc="right", ha="left", va="top")

plt.show()
