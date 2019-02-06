from random import uniform, random
from shutil import copyfile
from copy import copy
from os import system, makedirs, getpid
from itertools import chain
import argparse
import numpy as np

M_SHAPE = (4, 7)
N_COMM = 2

def call_dbg(prog, args):
    sargs = [str(i) for i in args]
    print(prog, " ".join(sargs))
    system(prog + " " + " ".join(sargs))

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
    return (avg_vel, confident_time)


def result_metric(result):
    # this limits the results to just Julio's time confidence metric
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

        # intentionally invalid, will be replaced at next minimize with self.Mx
        self.Mp = Evaluation(None, None, float('inf'))

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
        print("\t\tpar_id\n", self.pid_me)
        print("\t\tpid\n", getpid())
        print("result\n", self.Mx.result)
        print("r/b norm\n", self.Mx.result / base)
        print("Gr\n", Mg.result)
        print("r/Gr\n", self.Mx.result / Mg.result)
        print("M\n", self.Mx.M)
        print("V\n", self.Mv)
        print("G\n", Mg.M)

    def gravitate(self, Mg, w_i, w_l, w_g):
        for row in range(M_SHAPE[0]):
            for col in range(M_SHAPE[1]):
                if row > N_COMM - 1 and col > N_COMM - 1:
                    continue
                self.integrate_vel(row, col, Mg, w_i, w_l, w_g)

    def integrate_vel(self, r, c, Mg, w_i, w_l, w_g):
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

    def attempt_move(self, scene):
        self.Mx = Evaluation(self.Mv + self.Mx.M, scene)

    def minimize(self, Mg):
        if self.Mx < self.Mp:
            self.Mp = self.Mx
            if self.Mp < Mg:
                Mg = self.Mp
        return Mg


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

class Scene:
    def __init__(self, id, seed):
        self.id = id
        self.seed = seed

def simulate(M, trial, scene):
    trial_path = str(scene.id) + "/" + trial

    with open('data/comms.config', 'w') as config_clone:
        write_config(config_clone, M)
    copyfile('data/comms.config', "data/" + trial_path + ".config")

    call_dbg("build/bin/comm_norender", [scene.id, scene.seed])

    copyfile('data/comms.result', 'data/' + trial_path + ".result")
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
        Mg = p.minimize(Mg)
        p.debug(base.result, Mg)
        particles += [p]

    conv_path = str(scene["id"]) + "/convergence.txt"
    with open("data/" + conv_path, 'w') as f_conv:
        f_conv.write("convergence value per full iteration of all particles\n")
 
    enough_iter = 100
    enough_convergence = 1e-9
    for i in range(enough_iter):
        for p in particles:
            # integrate particle swarm dynamics
            p.gravitate(Mg, w_inertia, w_local, w_global)
            p.attempt_move(scene)

            # update known minima
            Mg = p.minimize(Mg)
            p.debug(base.result, Mg)

        # the potential variation of the fastest particle
        convergence = abs(max([np.linalg.norm(p.Mv) for p in particles]))
        with open("data/" + conv_path, 'a') as f_conv:
            f_conv.write("i: " + str(i) + ",  c: " + str(convergence)+"\n")
        if convergence < enough_convergence:
            break

    return particles

def run_scenario(run_name, scene, meta_parameters):
    makedirs("data/%s/%d_train/" % (run_name, scene["id"]))
    makedirs("data/%s/results/%d/" % (run_name, scene["id"]))
    PSO(scene, *meta_parameters)
    Evaluation.i = 0

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Globally optimize via Particle-Swarms the communication"
        "model for the Talking Hive project."
    )
    parser.add_argument("name", type=str)
    parser.add_argument("--n_particles", type=int, default=12)
    parser.add_argument("--w_inertia", type=float, default=0.2)
    parser.add_argument("--w_local", type=float, default=0.2)
    parser.add_argument("--w_global", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=511607575)

    args = parser.parse_args()

    for scene in chain(range(0, 10 + 1), range(14, 17 + 1)):
        # HERE, FORK THIS:
        run_scenario(
            args.run_name,
            Scene(scene, args.seed),
            [args.n_particles, args.w_inertia, args.w_local, args.w_global]
        )
    # JOIN HERE
