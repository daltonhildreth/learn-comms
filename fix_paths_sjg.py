import sys
from math import sqrt, pi, atan2, sin, cos

small_set =  [2,4,8,9,12,13,15,18]

dirs = []
for i in small_set:  # list(range(18 + 1)):
    for j in small_set:  # list(range(18 + 1)):
        dirs += ["data/%s/t%d_min/v%d/" % (sys.argv[1], i, j)]

for d in dirs:
    print(d)
    for prefix in ["", "b_"]:
        agents = dict()
        with open(d + prefix + "agent_paths.csv", "r") as og:
            with open(d + prefix + "fixed_paths.csv", "w") as fix:
                for line in og:
                    line = line.split(",")
                    id_ = int(line[0])
                    gid = int(line[1])
                    x = float(line[2])
                    y = float(line[3])
                    vx = float(line[4])
                    vy = float(line[5])
                    r = float(line[6])
                    t = float(line[7])
                    if id_ not in agents:
                        agents[id_] = [
                            {
                                "id": id_,
                                "x": x,
                                "y": y,
                                "ori": atan2(vy, vx),
                                "r": r,
                                "t": t,
                            }
                        ]
                    else:
                        s = sqrt(vx * vx + vy * vy)
                        alphaO = .8 + .2 * (1 - s)
                        if (s < .02): alphaO = 1
                        nvx = vx / s
                        nvy = vy / s
                        # npx = ???
                        # npy = ???
                        # score = ???
                        ordiff = atan2(vy, vx) - agents[id_][-1]["ori"] 
                        if (ordiff > pi):
                            ordiff -= 2 * pi
                        if (ordiff < -pi):
                            ordiff += 2 * pi
                        ori = (1.0 - alphaO) * ordiff + agents[id_][-1]["ori"]
                        if (ori > pi):
                            ori -= 2 * pi
                        if (ori < -pi):
                            ori += 2 * pi

                        agents[id_] += [
                            {
                                "id": id_,
                                "x": x,
                                "y": y,
                                "ori": ori,
                                "r": r,
                                "t": t,
                            }
                        ]

                    last = agents[id_][-1]
                    fix.write(
                        "%d,%d,%f,%f,%f,%f,%f,%f\n"
                        % (
                            last["id"],
                            last["id"],
                            last["x"],
                            last["y"],
                            cos(last["ori"]),
                            sin(last["ori"]),
                            last["r"],
                            last["t"],
                        )
                    )
