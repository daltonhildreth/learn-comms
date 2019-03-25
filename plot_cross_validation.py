import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import numpy as np

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import cmocean
import math
from mpl_toolkits.axes_grid1 import AxesGrid


def shiftedColorMap(cmap, start=0, midpoint=0.5, stop=1.0, name="shiftedcmap"):
    """
    Function to offset the "center" of a colormap. Useful for
    data with a negative min and positive max and you want the
    middle of the colormap's dynamic range to be at zero.

    Input
    -----
      cmap : The matplotlib colormap to be altered
      start : Offset from lowest point in the colormap's range.
          Defaults to 0.0 (no lower offset). Should be between
          0.0 and `midpoint`.
      midpoint : The new center of the colormap. Defaults to 
          0.5 (no shift). Should be between 0.0 and 1.0. In
          general, this should be  1 - vmax / (vmax + abs(vmin))
          For example if your data range from -15.0 to +5.0 and
          you want the center of the colormap at 0.0, `midpoint`
          should be set to  1 - 5/(5 + 15)) or 0.75
      stop : Offset from highest point in the colormap's range.
          Defaults to 1.0 (no upper offset). Should be between
          `midpoint` and 1.0.
    """
    cdict = {"red": [], "green": [], "blue": [], "alpha": []}

    # regular index to compute the colors
    reg_index = np.linspace(start, stop, 257)

    # shifted index to match the data
    shift_index = np.hstack(
        [
            np.linspace(0.0, midpoint, 128, endpoint=False),
            np.linspace(midpoint, 1.0, 129, endpoint=True),
        ]
    )

    for ri, si in zip(reg_index, shift_index):
        r, g, b, a = cmap(ri)

        cdict["red"].append((si, r, r))
        cdict["green"].append((si, g, g))
        cdict["blue"].append((si, b, b))
        cdict["alpha"].append((si, a, a))

    newcmap = mcolors.LinearSegmentedColormap(name, cdict)
    plt.register_cmap(cmap=newcmap)

    return newcmap


# biased_data = np.random.random_integers(low=-15, high=5, size=(37,37))

data = np.zeros((19, 19))
#
#
def read_cross_val(file):
    cvmin = float("inf")
    cross_val = []
    with open(file, "r") as f:
        f.readline()
        for line in f:
            line = line.split("\t")
            test = line[0]
            baseline = line[-1]
            norms = np.array(line[2:-1:2]).astype(float)
            cross_val += [norms]
            cvmin = min(cvmin, np.min(norms))
    print(cvmin)
    return cross_val


#
#
data = [
    np.array(read_cross_val("data/mar5_0341_audio/cross_val.tsv")),
    np.array(read_cross_val("data/mar5_1826_visual/cross_val.tsv")),
    np.array(read_cross_val("data/mar6_1445_both/cross_val.tsv")),
]

#
# fig = plt.figure()
# im = plt.imshow(
#    data,
#    extent=(0, 18, 0, 18),
#    interpolation="nearest",
#    cmap="RdGy_r",
#    norm=colors.LogNorm(vmin=0.05, vmax=20),
# )
# plt.xticks(np.arange(19))
# plt.yticks(np.arange(19))
#

# sample the colormaps that you want to use. Use 128 from each so we get 256
# colors in total
colors1 = plt.cm.viridis(np.linspace(0.15, 0.8, 128)) # 0.25, 0.9
colors2 = plt.cm.plasma_r(np.linspace(0.0, 0.65, 128)) # 0.1, 0.75

# combine them and build a new colormap
colors = np.vstack((colors1, colors2))
orig_cmap = mcolors.LinearSegmentedColormap.from_list("my_colormap", colors)

# shifted_cmap = shiftedColorMap(orig_cmap, midpoint=0.5, name='shifted')
# shrunk_cmap = shiftedColorMap(orig_cmap, start=0.15, midpoint=0.5, stop=.85, name='shrunk')

fig = plt.figure(figsize=(6, 6))
# grid = AxesGrid(
#    fig,
#    111,
#    nrows_ncols=(2, 2),
#    axes_pad=0.5,
#    label_mode="1",
#    share_all=True,
#    cbar_location="right",
#    cbar_mode="each",
#    cbar_size="7%",
#    cbar_pad="2%",
# )

# normal cmap
im0 = plt.imshow(
    data[2],
    interpolation="none",
    cmap=orig_cmap,
    norm=mcolors.LogNorm(vmax=10, vmin=.1),
    # vmax=1.4,
    # vmin=-1.4,
)
cbar = plt.colorbar(im0, ticks=[0.1, 0.2, 0.5, 1, 2, 5, 10])
cbar.ax.set_yticklabels(
    [
        ("%sx") % (("%f" % i).rstrip("0").rstrip("."))
        for i in [0.1, 0.2, 0.5, 1, 2, 5, 10]
    ]
)
plt.xticks(np.arange(0, 19, 1))
plt.xlabel("Model")
plt.yticks(np.arange(0, 19, 1))
plt.ylabel("Test")
plt.title("Audiovisual")

# im1 = grid[1].imshow(
#    data[1],
#    interpolation="none",
#    cmap=orig_cmap,
#    norm=mcolors.LogNorm(vmax=20, vmin=0.05),
#    #vmax=1.4,
#    #vmin=-1.4,
# )
# grid.cbar_axes[1].colorbar(im1)
# grid[1].set_title("visual", fontsize=8)
#
# im2 = grid[2].imshow(
#    data[2],
#    interpolation="none",
#    cmap=orig_cmap,
#    norm=mcolors.LogNorm(vmax=20, vmin=0.05),
#    #vmax=1.4,
#    #vmin=-1.4,
# )
# grid.cbar_axes[2].colorbar(im2)
# grid[2].set_title("both", fontsize=8)
#
# for ax in grid:
#    ax.set_yticks([])
#    ax.set_xticks([])

plt.show()
