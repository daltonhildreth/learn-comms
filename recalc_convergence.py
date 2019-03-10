import numpy as np

# this was first done:
# awk 'NR % 25 > 8 && NR % 25 < 13' audio/tN_train/opt.log > audio/tN_train/opt_xM
for N in range(18 + 1):
    with open("data/mar5_0341_audio/t%d_train/convergence.log" % N, "w") as conv:
        conv.write("convergence value per full iteration of all particles\n")
    with open("data/mar5_0341_audio/t%d_train/opt_xM" % N, "r") as opt:
        for it in range(500):
            positions = []
            for p in range(40):
                nxt = []
                nxt += [np.array(opt.readline().split()).astype(float)]
                nxt += [np.array(opt.readline().split()).astype(float)]
                nxt += [np.array(opt.readline().split()).astype(float)]
                nxt += [np.array(opt.readline().split()).astype(float)]
                positions += [np.array(nxt)]
            # positions = [p.pos.config for p in particles]
            q3, q1 = np.percentile(positions, [75, 25], axis=0)
            convergence = np.mean(q3 - q1)
            with open("data/mar5_0341_audio/t%d_train/convergence.log" % N, "a") as conv:
                conv.write("i: %d, c: %f\n" % (it, convergence))
