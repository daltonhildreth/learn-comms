from os import listdir
from itertools import chain

def minimize(directory):
    min_val = float('inf')
    min_file = ""
    for filename in listdir(directory):
        if ".result" in filename:
            # grab the # of t#.result
            i = int(filename.split('.')[0][1:])

            # skip baseline
            if i == 0:
                 continue

            with open(directory + filename, "r") as f:
                avg_vel = float(f.readline().strip())
                confident_time = float(f.readline().strip())
                val = confident_time

                if val < min_val:
                    min_val = val
                    min_file = filename
    return (min_file, min_val)

for scene in chain(range(0, 10+1), range(14, 17+1)):
    print(str(scene), minimize("data/"+str(scene)+"/"))
