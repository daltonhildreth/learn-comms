import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import math

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

kind = "mar27_1426_circ_audio_noiters"#"mar5_1826_visual"#"mar6_1445_both"
nits = 225

# separate plots
fig, ax = plt.subplots(figsize=(12,6))#nrows=19, ncols=1, figsize=(3, 9))
#fig.tight_layout()
#ax[0] = plt.subplot2grid((3, 20), (0, 0), colspan=20)
#ax[1] = plt.subplot2grid((3, 20), (1, 0), colspan=9)
#ax[2] = plt.subplot2grid((3, 20), (2, 0), colspan=9)
#for sub, kind in enumerate(
#    ["mar5_0341_audio", "mar5_1826_visual", "mar6_1445_both"]
#):

max_c = -float('inf')
min_c = float('inf')
for plot in range(18 + 1):
    # separate lines
    particles = []
    its = np.arange(0, nits + 1, 1)
    for p in range(40):
        particles += [[]]
    with open("data/%s/t%d_train/opt_xrb" % (kind, plot), "r") as f:
        for p, line in enumerate(f):
            c = float(line.split()[-1])
            particles[p % 40] += [c]

    particles = list(zip(*particles))
    iqr = []
    for i in its:
        # plot min
        # plot median
        # plot max
        q2 = np.percentile(particles[i], 0, axis=0)
        #s = np.mean(particles[i] - np.ones((40,))*q2)
        iqr += [math.log(abs(q2), 10)]
        max_c = max(max_c, iqr[-1])
        min_c = min(min_c, iqr[-1])
    window = 1
    for i in its[window:]:
        iqr[i] = np.mean(iqr[i-window:i+1])

    ax.plot(its[window:], np.array(iqr[window:]), linewidth=1, color=scn_colors[plot])

    ax.margins(x=0)
    c10 = (max_c - min_c) / 16.
    cr = np.arange(min_c, (max_c+c10), c10)
    ir = np.arange(its[window], its[-1]+25, 25)
    ax.set_yticks(cr)
    ax.set_yticklabels(["%.2f" % i for i in cr])
    ax.set_xticks(ir)
    ax.set_xticklabels(ir, rotation=90)
    #ax.set_title("%d" % plot, loc="right", ha="left", va="top")
ax.legend(['%d' % plot for plot in range(19)], loc = "right")

plt.show()
