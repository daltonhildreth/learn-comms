from os import listdir, makedirs, path
from os.path import isdir
from shutil import copyfile
from itertools import chain
from sys import argv

def read_result(file):
    with open(file, "r") as f:
        avg_vel = float(f.readline().strip())
        confident_time = float(f.readline().strip())
        val = confident_time
        return val

def minimize(directory):
    min_val = float('inf')
    base_val = 0
    min_file = ""
    for iteration in listdir(directory):
        iteration = directory + iteration + "/"
        if not isdir(iteration):
            continue
        for filename in listdir(iteration):
            if ".result" not in filename:
                continue
            # grab the # of #_p#.result
            sim = filename.split('.')[0].split("_p")
            i = int(sim[0])
            p = int(sim[1]) if len(sim) > 1 else None

            # skip baseline
            if i == 0:
                base_val = read_result(iteration + filename)
                continue

            val = read_result(iteration + filename)
            if val < min_val:
                min_val = val
                min_file = filename.split('.')[0]
    return (base_val, min_file, min_val)

if __name__ == "__main__":

    for scene in [14]:#chain(range(0, 10+1), range(14, 17+1)):
        src = "data/%s/%d_train/iters/" % (argv[1], scene)
        dst = "data/%s/results/%d_min/" % (argv[1], scene)
        base_val, min_file, min_val = minimize(src)
        makedirs(dst)
        with open("data/%s/results/best.txt" % argv[1], "a") as best:
            best.write(
                str(scene)
                + "\tbest at config: " + min_file
                + "\tw/ conf_time: " + str(min_val)
                + "\tvs base conf_time: " + str(base_val)
                + "\n"
            )
        copyfile(
            src + min_file + "/" + min_file + ".config",
            dst + "comms.config"
        )
        copyfile(
            src + min_file + "/" + min_file + ".result",
            dst + min_file + ".result"
        )
