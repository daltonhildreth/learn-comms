#Globally Optimize by using covariance matrix adaption:
#
from random import uniform, random
from shutil import copyfile
from copy import copy
from os import system
import argparse
import numpy as np

N_PARTICLES = 12
W_INERTIA = 0.2
W_LOCAL = 0.2
W_GLOBAL = 0.2
IS_COMM = False
IS_RENDER = False
SEED = 5117
M_SHAPE = (3,5)

def write_config(file, config):
    file.write(M_SHAPE[0] + " " + M_SHAPE[1] + "\n")
    for i in range(0, M_SHAPE[0]):
        for j in range(0, M_SHAPE[1]):
            file.write(str(config[i][j]))
            if (j != 4):
                file.write(' ')
        file.write('\n')

#will get results to influence nn choice from M
def read_result(file):
    avg_vel = float(file.readline().strip())
    confident_time = float(file.readline().strip())
    #avg_time =
    return (avg_vel, confident_time)

def result_metric(result):
    _, conf_time = result
    return conf_time

class Particle:
    def __init__(self, Blo, Bhi):
        self.Mx = np.empty(shape=M_SHAPE)
        for row in range(len(self.Mx)):
            for col in range(len(self.Mx[row])):
                self.Mx[row][col] = uniform(Blo[row][col], Bhi[row][col])
        self.Mx = Evaluation(self.Mx)

        self.Mp = self.Mx

        self.Mv = np.empty(shape=M_SHAPE)
        for row in range(len(self.Mv)):
            for col in range(len(self.Mv[row])):
                mag = abs(Bhi[row][col] - Blo[row][col])
                self.Mv[row][col] = uniform(-mag, mag)

    def debug(self, base, Mg):
        print("result\n": self.Mx.result)
        print("norm\n", self.Mx.result / base)
        print("ratio\n", self.Mx.result / Mg.result)
        print("M\n", self.Mx.M)
        print("V\n", self.Mv.M)
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

    def minimize(self, Mg)
        if self.Mx < self.Mp:
            self.Mp = self.Mx
            if self.Mp < Mg:
                Mg = self.Mp


class Evaluation:
    def __init__(self, M, result=None):
        self.M = M
        if result == None:
            self.result = simulate(config=M)
        else:
            self.result = result

    def __gt__(self, other):
        return self.result > other.result

    def __lt__(self, other):
        return self.result < other.result

def simulate(M, trial):
    subset = ("" if IS_COMM else "no")+"comm_"+\
        ("" if IS_RENDER else "no")+"render/"

    with open('data/comms.config', 'w') as config_clone:
        write_config(config_clone, M)
    copyfile('data/comms.config', 'data/' + subset + trial + ".config")

    prog = "build/" + config + "bin/gg-engine"
    args = [str(SEED)]
    system(prog + " " + " ".join(args))

    copyfile("data/comms.result", "data/" + subset + trial + ".result")

    return result_metric(read_result(""))

def PSO(n, shape, w_inertia=0.2, w_local=0.2, w_global=0.2):
    particles = [None] * N_PARTICLES
    Mg = Evaluation(np.zeros(shape=M_SHAPE))
    Blo = -np.ones(shape=M_SHAPE)
    Bhi = np.ones(shape=M_SHAPE)

    for p in particles:
        p = Particle(Blo, Bhi)

        # don't p.minimize() because all Mp == Mx
        if p.Mp < Mg:
            Mg = p.Mp

    convergent = False
    enough_iter = False
    i = 0
    enough = 200
    while not (convergent or enough_iter):
        for p in particles:
            # accelerate / explore
            for row in range(M_SHAPE[0]):
                for col in range(M_SHAPE[1]):
                    p.update_vel(row, col, Mg, w_inertia, w_local, w_global)

            # integrate velocity
            p.Mx = Evaluation(p.Mv + p.Mx.M)

            # update minima
            p.minimize(Mg)
            p.debug()
        enough_iter = i >= enough
        convergent = False #.... TODO

    return (particles, convergent, enough_iter)

#for each particle i = 1, ..., S do
#   Initialize the particle's position with a uniformly distributed random vector: xi ~ U(blo, bup)
#   Initialize the particle's best known position to its initial position: pi ← xi
#   if f(pi) < f(g) then
#       update the swarm's best known  position: g ← pi
#   Initialize the particle's velocity: vi ~ U(- | bup - blo | , | bup-blo|)
#while a termination criterion is not met do:
#   for each particle i = 1, ..., S do
#      for each dimension d = 1, ..., n do
#         Pick random numbers: rp, rg ~ U(0, 1)
#         Update the particle's velocity: vi, d ← ω vi, d + φp rp(pi, d - xi, d) + φg rg(gd - xi, d)
#      Update the particle's position: xi ← xi + vi
#      if f(xi) < f(pi) then
#         Update the particle's best known position: pi ← xi
#         if f(pi) < f(g) then
#            Update the swarm's best known position: g ← pi


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="\
        Globally optimize via Particle-Swarms the linear communication model\n\
        for the Talking Hive project.\n")
    parser.add_argument("n_particles", metavar="N_PARTICLES")
    parser.add_argument("w_inertia", metavar="W_INERTIA")
    parser.add_argument("w_local", metavar="W_LOCAL")
    parser.add_argument("w_global", metavar="W_GLOBAL")
    parser.add_argument("is_comm", metavar="IS_COMM")
    parser.add_argument("is_render", metavar="IS_RENDER")
    parser.add_argument("seed", metavar="SEED")

    pass
