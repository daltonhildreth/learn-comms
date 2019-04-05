import sys
from math import sqrt

small_set = [2,9]#[2,4,8,9,12,13,15,18]

dirs = []
for i in small_set:  # list(range(18 + 1)):
    for j in small_set:  # list(range(18 + 1)):
        dirs += ["data/%s/t%d_min/v%d/" % (sys.argv[1], i, j)]

for d in dirs:
    print(d)
    for prefix in ["", "b_"]:
        agents = dict()
        with open(d + prefix + "agent_paths.csv", "r") as og:
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
                            "vx": vx,
                            "vy": vy,
                            "r": r,
                            "t": t,
                        }
                    ]
                else:
                    agents[id_] += [
                        {
                            "id": id_,
                            "x": x,
                            "y": y,
                            "vx": vx,
                            "vy": vy,
                            "r": r,
                            "t": t,
                        }
                    ]
        timestamps = len(agents[1])
        for agent in agents:
            agent = agents[agent]
            for i, time in enumerate(agent):
                window = 1
                start = max(0, i - window)
                end = min(timestamps - 1, i + window)
                sx = agent[start]["x"]
                sy = agent[start]["y"]
                ex = agent[end]["x"]
                ey = agent[end]["y"]
                dt = (end - start) / 60.0

                vx = 0
                vy = 0
                a_window = agent[start : end + 1]
                for u, v in zip(a_window, a_window[1:]):
                    vx += ((v["x"] - u["x"]) * dt) / len(a_window)
                    vy += ((v["y"] - u["y"]) * dt) / len(a_window)

                #vx = (ex - sx) * dt
                #vy = (ey - sy) * dt

                s = sqrt(vx * vx + vy * vy)
                time["nvx"] = (
                    vx / s
                    if s > 0.02
                    else (agent[i + 1]["vx"] if i == 0 else agent[i - 1]["nvx"])
                )
                time["nvy"] = (
                    vy / s
                    if s > 0.02
                    else (agent[i + 1]["vy"] if i == 0 else agent[i - 1]["nvy"])
                )

        with open(d + prefix + "fixed_paths.csv", "w") as fix:
            for i in range(timestamps):
                for a in agents:
                    a = agents[a]
                    fix.write(
                        "%d,%d,%f,%f,%f,%f,%f,%f\n"
                        % (
                            a[i]["id"],
                            a[i]["id"],
                            a[i]["x"],
                            a[i]["y"],
                            a[i]["nvx"],
                            a[i]["nvy"],
                            a[i]["r"],
                            a[i]["t"],
                        )
                    )
