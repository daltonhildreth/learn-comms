# Globally Optimize by using covariance matrix adaption:
#
from random import uniform, random
from shutil import copyfile
from copy import copy
from os import system
import argparse
import numpy as np

M_SHAPE = (4, 7)
N_COMM = 2

def write_config(file, config):
    file.write(str(M_SHAPE[0]) + " " + str(M_SHAPE[1]) + "\n")
    for i in range(0, M_SHAPE[0]):
        for j in range(0, M_SHAPE[1]):
            file.write(str(config[i][j]))
            if (j != M_SHAPE[1] - 1):
                file.write(' ')
        file.write('\n')


def read_result(file):
    # will get results to influence nn choice from M
    avg_vel = float(file.readline().strip())
    confident_time = float(file.readline().strip())
    # avg_time =
    return (avg_vel, confident_time)


def result_metric(result):
    _, conf_time = result
    return conf_time


class Particle:
    pid = 0

    def __init__(self, Blo, Bhi, scene):
        self.pid_me = Particle.pid
        Particle.pid += 1

        self.Mx = np.zeros(shape=M_SHAPE)
        for row in range(len(self.Mx)):
            for col in range(len(self.Mx[row])):
                # Don't fill in values that are redundant to TTC
                if row > N_COMM - 1 and col > N_COMM - 1:
                    continue
                self.Mx[row][col] = uniform(Blo[row][col], Bhi[row][col])
        self.Mx = Evaluation(self.Mx, scene)

        self.Mp = self.Mx

        self.Mv = np.zeros(shape=M_SHAPE)
        for row in range(len(self.Mv)):
            for col in range(len(self.Mv[row])):
                # Don't fill in values that are redundant to TTC
                if row > N_COMM - 1 and col > N_COMM - 1:
                    continue
                mag = abs(Bhi[row][col] - Blo[row][col])
                self.Mv[row][col] = uniform(-mag, mag)

    def debug(self, base, Mg):
        print("\t\ti\n", Evaluation.i)
        print("\t\tpid\n", self.pid_me)
        print("result\n", self.Mx.result)
        print("r/b norm\n", self.Mx.result / base)
        print("Gr\n", Mg.result)
        print("r/Gr\n", self.Mx.result / Mg.result)
        print("M\n", self.Mx.M)
        print("V\n", self.Mv)
        print("G\n", Mg.M)

    def update_vel(self, r, c, Mg, w_i, w_l, w_g):
        local_flux = random()
        global_flux = random()

        vel = self.Mv[r][c]
        local_min = self.Mp.M[r][c]
        global_min = Mg.M[r][c]
        pos = self.Mx.M[r][c]

        momentum = w_i * vel
        local_gravity = w_l * local_flux * (local_min - pos)
        global_gravity = w_g * global_flux * (global_min - pos)

        self.Mv[r][c] = momentum + local_gravity + global_gravity

    def minimize(self, Mg):
        if self.Mx < self.Mp:
            self.Mp = self.Mx
            if self.Mp < Mg:
                Mg = self.Mp


class Evaluation:
    i = 0

    def __init__(self, M, scene, result=None):
        self.M = M
        self.scene = scene
        if result == None:
            self.result = simulate(M, "t" + str(self.i), scene)
            Evaluation.i += 1
        else:
            self.result = result

    def __gt__(self, other):
        return self.result > other.result

    def __lt__(self, other):
        return self.result < other.result

    def debug(self):
        print("result\n", self.result)
        print("M\n", self.M)


def simulate(M, trial, scene):
    subset = ("" if scene["with_comm"] else "no") + "comm_" +\
        ("" if scene["with_render"] else "no") + "render"

    with open('data/comms.config', 'w') as config_clone:
        write_config(config_clone, M)
    copyfile(
        'data/comms.config', 'data/' + str(scene["id"])
        + "/" + subset + "/" + trial + ".config"
    )

    prog = "build/bin/" + subset
    args = [str(scene["id"]), str(scene["seed"])]
    print(prog, " ".join(args))
    system(prog + " " + " ".join(args))

    copyfile(
        "data/comms.result", "data/" + str(scene["id"]) + "/"
        + subset + "/" + trial + ".result"
    )

    with open('data/comms.result', 'r') as result_clone:
        return result_metric(read_result(result_clone))


def PSO(scene, n, w_inertia=0.2, w_local=0.2, w_global=0.2):
    particles = []  # [None] * n
    base_scene = copy(scene)
    #base_scene["with_comm"] = False
    base = Evaluation(np.zeros(shape=M_SHAPE), base_scene)
    base.debug()
    Mg = Evaluation(np.empty(0), scene, result=float('inf'))
    Blo = -np.ones(shape=M_SHAPE)
    Bhi = np.ones(shape=M_SHAPE)

    for p in range(n):
        p = Particle(Blo, Bhi, scene)

        # don't p.minimize() because all Mp == Mx
        if p.Mp < Mg:
            Mg = p.Mp
        p.debug(base.result, Mg)
        particles += [p]

    convergent = False
    enough_iter = False
    i = 0
    enough = 100
    while not (convergent or enough_iter):
        for p in particles:
            # accelerate / explore
            for row in range(M_SHAPE[0]):
                for col in range(M_SHAPE[1]):
                    if row > 0 and col > 0:
                        continue
                    p.update_vel(row, col, Mg, w_inertia, w_local, w_global)

            # integrate velocity
            p.Mx = Evaluation(p.Mv + p.Mx.M, scene)

            # update minima
            # p.minimize(Mg)
            if p.Mx < p.Mp:
                p.Mp = p.Mx
                if p.Mp < Mg:
                    Mg = p.Mp
            p.debug(base.result, Mg)
        enough_iter = i >= enough
        i += 1
        # convergent = False  # TODO
        convergent = max(sum([np.linalg.norm(p.Mv) for p in particles])) < 1e-9

    return (particles, convergent, enough_iter)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="\
        Globally optimize via Particle-Swarms the linear communication model\n\
        for the Talking Hive project.\n")
    parser.add_argument("--n_particles", type=int, default=12)
    parser.add_argument("--w_inertia", type=float, default=0.2)
    parser.add_argument("--w_local", type=float, default=0.2)
    parser.add_argument("--w_global", type=float, default=0.2)
    parser.add_argument("--is_comm", type=bool, default=True)
    parser.add_argument("--is_render", type=bool, default=False)
    parser.add_argument("--seed", type=int, default=511607575)

    args = parser.parse_args()

    for scene in range(0, 10 + 1):
        PSO(
            {
                "id": scene,
                "seed": args.seed,
                "with_comm": args.is_comm,
                "with_render": args.is_render
            },
            args.n_particles, args.w_inertia, args.w_local, args.w_global
        )
    for scene in range(14, 17 + 1):
        PSO(
            {
                "id": scene,
                "seed": args.seed,
                "with_comm": args.is_comm,
                "with_render": args.is_render
            },
            args.n_particles, args.w_inertia, args.w_local, args.w_global
        )
