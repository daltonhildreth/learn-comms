import os

def minimize(directory):
    minv = float('inf')
    minf = ""
    for filename in os.listdir(directory):
        if (".result" in filename):
            i = int(filename.split('.')[0][1:])
            if i == 0:
                 continue
            with open(directory + filename, "r") as f:
                f.readline()
                r = f.readline().strip()
                if minv > float(r):
                    minv = float(r)
                    minf = filename
    return (minf, minv)

for scene in range(0, 10+1):
    print(minimize("data/"+str(scene)+"/comm_norender/"))
for scene in range(14, 17+1):
    print(minimize("data/"+str(scene)+"/comm_norender/"))
