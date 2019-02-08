from random import uniform, random
from shutil import copyfile
from copy import copy
from os import mkdir, makedirs, getpid
from subprocess import Popen, PIPE
from itertools import chain
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
    return result_metric((avg_vel, confident_time))


def result_metric(result):
    # this limits the results to just Julio's time confidence metric
    _, confident_time = result
    return confident_time


class Particle:
    def __init__(self, id_, min_config, max_config, scene, data_dir):
        self.id = id_
        # the baseline always gets 0
        self.iters = 1
        self.scene = scene
        self.data_dir = data_dir
        self.save = "%d_p%d" % (self.iters, self.id)

        spawn_config = np.zeros(shape=M_SHAPE)
        for r in range(len(spawn_config)):
            for c in range(len(spawn_config[r])):
                # Don't fill in values that are redundant to TTC
                if r > N_COMM - 1 and c > N_COMM - 1:
                    continue
                spawn_config[r][c] = uniform(min_config[r][c], max_config[r][c])
        self.pos = simulate(self.scene, spawn_config, self.data_dir, self.save)

        # intentionally invalid, will be replaced at next minimize with self.pos
        self.local_min = Point()

        self.vel = np.zeros(shape=M_SHAPE)
        for r in range(len(self.vel)):
            for c in range(len(self.vel[r])):
                # Don't fill in values that are redundant to TTC
                if r > N_COMM - 1 and c > N_COMM - 1:
                    continue
                mag = abs(max_config[r][c] - min_config[r][c])
                self.vel[r][c] = uniform(-mag, mag)

    def debug(self, base, global_min):
        print("\t\ti ", self.iters)
        print("\t\tparticle id ", self.id)
        print("\t\tprocess id ", getpid())
        print("xr ", self.pos.score)
        print("norm xr/b ", self.pos.score / base.score)
        print("Gr ", global_min.score)
        print("norm xr/Gr ", self.pos.score / global_min.score)
        print("xM\n", self.pos.config)
        print("vM\n", self.vel)
        print("gM\n", global_min.config)

    def gravitate(self, global_min, hypers):
        for r in range(M_SHAPE[0]):
            for c in range(M_SHAPE[1]):
                if r > N_COMM - 1 and c > N_COMM - 1:
                    continue
                self.integrate_vel(r, c, global_min, hypers)

    def integrate_vel(self, r, c, global_min, hypers):
        local_flux = random()
        global_flux = random()

        vel = self.vel[r][c]
        local_min = self.local_min.config[r][c]
        global_min = global_min.config[r][c]
        pos = self.pos.config[r][c]

        momentum = hypers.w_inertia * vel
        local_gravity = hypers.w_local * local_flux * (local_min - pos)
        global_gravity = hypers.w_global * global_flux * (global_min - pos)

        self.vel[r][c] = momentum + local_gravity + global_gravity

    def attempt_move(self):
        self.iters += 1
        self.pos = simulate(
            self.scene, self.vel + self.pos.config, self.data_dir, self.save
        )

    def minimize(self, global_min):
        if self.pos.score < self.local_min.score:
            self.local_min = self.pos
            if self.local_min.score < global_min.score:
                global_min = self.local_min
        return global_min

class Point:
    def __init__(self, x=None, y=float('inf')):
        self.config = x
        self.score = y

class Scene:
    def __init__(self, id_, seed):
        self.id = id_
        self.seed = seed

class HyperParameters:
    def __init__(
        self, n_particles, w_inertia, w_local, w_global, max_iters,
        max_convergence
    ):
        self.n_particles = n_particles
        self.w_inertia = w_inertia
        self.w_local = w_local
        self.w_global = w_global
        self.max_iters = max_iters
        self.max_convergence = max_convergence

def simulate(scene, config, data_dir, save_path):
    prog = "build/bin/comm_norender"
    # the 0 does not matter since we are using a _norender build
    args = [str(i) for i in [scene.id, scene.seed, 0, data_dir]]
    data_dir = "data/" + data_dir
    save = (data_dir, save_path)

    sim_config = "%s/comms.config" % data_dir
    with open(sim_config, 'w') as config_file:
        write_config(config_file, config)
    copyfile(sim_config, "%s/%s.config" % save)

    print(" ".join([prog] + args))
    p = Popen([prog] + args) # yield after this?

    sim_result = "%s/comms.result" % data_dir
    copyfile(sim_result, "%s/%s.result" % save)
    with open(sim_result, 'r') as result_clone:
        return Point(config, read_result(result_clone))


def PSO(data_dir, scene, hypers):
    base_scn = copy(scene)
    baseline = simulate(base_scn, np.zeros(shape=M_SHAPE), data_dir, str(0))
    # === SIMULATE BLOCKS HERE ===
    print("baseline ==== ", baseline.score)
    global_min = Point()

    # initialize particles
    min_config = -np.ones(shape=M_SHAPE)
    max_config = np.ones(shape=M_SHAPE)
    particles = []
    for p in range(hypers.n_particles):
        p = Particle(p, min_config, max_config, scene, data_dir)
        # === SIMULATE BLOCKS HERE ===
        global_min = p.minimize(global_min)
        p.debug(baseline, global_min)
        particles += [p]

    conv_path = data_dir + "/convergence.txt"
    with open("data/" + conv_path, 'w') as f_conv:
        f_conv.write("convergence value per full iteration of all particles\n")
 
    for i in range(hypers.max_iters):
        for p in particles:
            # integrate particle swarm dynamics
            p.gravitate(global_min, hypers)
            p.attempt_move()
            # === SIMULATE BLOCKS HERE ===

            # update known minima
            global_min = p.minimize(global_min)
            p.debug(baseline, global_min)

        # the potential variation of the fastest particle
        convergence = abs(max([np.linalg.norm(p.vel) for p in particles]))
        with open("data/" + conv_path, 'a') as f_conv:
            f_conv.write("i: " + str(i) + ",  c: " + str(convergence)+"\n")
        if convergence < hypers.max_convergence:
            return

def run_scenario(run_name, scene, meta_args):
    data_dir = "%s/%d_train" % (run_name, scene.id)
    makedirs("data/" + data_dir)
    PSO(data_dir, scene, meta_args)

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
    parser.add_argument("--iters", type=int, default=100)
    parser.add_argument("--convergence", type=float, default=1e-9)
    args = parser.parse_args()

    # High-Level what should be going on.
    # while optimizing:
    #    for scene in scenes:
    #        run(scene):
    #            #...
    #            next(PSO(scene)):
    #                #...
    #                R = yield from simulate(scene):
    #                    #...
    #                    o = Popen
    #                    while o.poll() is not None:
    #        /---------------yield None
    #        \-------------->
    #                ^       #...
    #                \---yield r
    #                while iterating:
    #                    for p in particles:
    #                        #...
    #                        p.R = yield from simulate(scene):
    #                            #...
    #                            o = Popen
    #                            while o.poll() is not None:
    #        /-----------------------yield None
    #        \---------------------->
    #                        ^   #...
    #                        \---yield r
    #                    #...
    #                #...
    #            #...
    for scene in chain(range(0, 10 + 1), range(14, 17 + 1)):
        run_scenario(
            args.name,
            Scene(scene, args.seed),
            HyperParameters(
                args.n_particles,
                args.w_inertia,
                args.w_local,
                args.w_global,
                args.iters,
                args.convergence
            )
        )
