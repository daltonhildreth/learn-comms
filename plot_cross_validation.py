import matplotlib.pyplot as plt
import matplotlib.colors as colors
import numpy as np

np.random.seed(19680801)
data = np.random.uniform(low=-1, high=1, size=(19, 19))

def read_cross_val(file):
    cross_val = []
    with open(file, "r") as f:
        f.readline()
        for line in f:
            line = line.split("\t")
            test = line[0]
            baseline = line[-1]
            norms = np.array(line[2:-1:2]).astype(float)
            cross_val += [norms]
    return cross_val

data = np.array(
#read_cross_val("data/mar5_0341_audio/cross_val.tsv")
#read_cross_val("data/mar5_1826_visual/cross_val.tsv")
read_cross_val("data/mar6_1445_both/cross_val.tsv")
)

fig = plt.figure()
im = plt.imshow(data, extent=(0, 18, 0, 18),
    interpolation='nearest', cmap='PiYG_r', norm = colors.LogNorm(vmin=0.05,vmax=20))
plt.xticks(np.arange(19))
plt.yticks(np.arange(19))

fig.colorbar(im)

plt.show()
