from random import uniform, random
from shutil import copyfile
from copy import copy
from os import mkdir, makedirs, getpid
import subprocess
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
    total_id = 0

    def __init__(self, min_config, max_config, scene):
        self.id = Particle.total_id
        Particle.total_id += 1

        spawn_config = np.zeros(shape=M_SHAPE)
        for r in range(len(spawn_config)):
            for c in range(len(spawn_config[r])):
                # Don't fill in values that are redundant to TTC
                if r > N_COMM - 1 and c > N_COMM - 1:
                    continue
                spawn_config[r][c] = uniform(min_config[r][c], max_config[r][c])
        self.pos = Evaluation(spawn_config, scene)

        # intentionally invalid, will be replaced at next minimize with self.Mx
        self.local_min = Evaluation(None, None, float('inf'))

        self.vel = np.zeros(shape=M_SHAPE)
        for r in range(len(self.vel)):
            for c in range(len(self.vel[r])):
                # Don't fill in values that are redundant to TTC
                if r > N_COMM - 1 and c > N_COMM - 1:
                    continue
                mag = abs(max_config[r][c] - min_config[r][c])
                self.vel[r][c] = uniform(-mag, mag)

    def debug(self, base, global_min):
        print("\t\ti ", Evaluation.i)
        print("\t\tparticle id ", self.id)
        print("\t\tprocess id ", getpid())
        print("xr ", self.pos.result)
        print("norm xr/b ", self.pos.result / base)
        print("Gr ", global_min.result)
        print("norm xr/Gr ", self.pos.result / global_min.result)
        print("xM\n", self.pos.config)
        print("vM\n", self.vel)
        print("gM\n", global_min.config)

    def gravitate(self, global_min, meta_args):
        for r in range(M_SHAPE[0]):
            for c in range(M_SHAPE[1]):
                if r > N_COMM - 1 and c > N_COMM - 1:
                    continue
                self.integrate_vel(r, c, global_min, meta_args)

    def integrate_vel(self, r, c, global_min, meta_args):
        local_flux = random()
        global_flux = random()

        vel = self.vel[r][c]
        local_min = self.local_min.config[r][c]
        global_min = global_min.config[r][c]
        pos = self.pos.config[r][c]

        momentum = meta_args.w_inertia * vel
        local_gravity = meta_args.w_local * local_flux * (local_min - pos)
        global_gravity = meta_args.w_global * global_flux * (global_min - pos)

        self.vel[r][c] = momentum + local_gravity + global_gravity

    def attempt_move(self, scene):
        self.pos = Evaluation(self.vel + self.pos.config, scene)

    def minimize(self, global_min):
        if self.pos < self.local_min:
            self.local_min = self.pos
            if self.local_min < global_min:
                global_min = self.local_min
        return global_min


class Evaluation:
    i = 0
    data_dir = ""

    def __init__(self, config, scene, result=None):
        self.config = config
        self.scene = scene
        if result == None:
            self.result = simulate(scene, config, self.data_dir, self.i)
            Evaluation.i += 1
        else:
            self.result = result

    def __gt__(self, other):
        return self.result > other.result

    def __lt__(self, other):
        return self.result < other.result

    def debug(self):
        print("result\n", self.result)
        print("config\n", self.config)

class Scene:
    def __init__(self, id, seed):
        self.id = id
        self.seed = seed

class MetaParameters:
    def __init__(self, n_particles, w_inertia, w_local, w_global):
        self.n_particles = n_particles
        self.w_inertia = w_inertia
        self.w_local = w_local
        self.w_global = w_global

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
    subprocess.call([prog] + args)

    sim_result = "%s/comms.result" % data_dir
    copyfile(sim_result, "%s/%s.result" % save)
    with open(sim_result, 'r') as result_clone:
        return read_result(result_clone)


def PSO(scene, meta_args):
    particles = []  # [None] * n
    base_scene = copy(scene)
    #base_scene["with_comm"] = False
    base = Evaluation(np.zeros(shape=M_SHAPE), base_scene)
    base.debug()
    global_min = Evaluation(None, None, result=float('inf'))
    min_config = -np.ones(shape=M_SHAPE)
    max_config = np.ones(shape=M_SHAPE)

    for p in range(meta_args.n_particles):
        p = Particle(min_config, max_config, scene)
        global_min = p.minimize(global_min)
        p.debug(base.result, global_min)
        particles += [p]

    conv_path = Evaluation.data_dir + "/convergence.txt"
    with open("data/" + conv_path, 'w') as f_conv:
        f_conv.write("convergence value per full iteration of all particles\n")
 
    enough_iter = 100
    enough_convergence = 1e-9
    for i in range(enough_iter):
        for p in particles:
            # integrate particle swarm dynamics
            p.gravitate(global_min, meta_args)
            p.attempt_move(scene)

            # update known minima
            global_min = p.minimize(global_min)
            p.debug(base.result, global_min)

        # the potential variation of the fastest particle
        convergence = abs(max([np.linalg.norm(p.vel) for p in particles]))
        with open("data/" + conv_path, 'a') as f_conv:
            f_conv.write("i: " + str(i) + ",  c: " + str(convergence)+"\n")
        if convergence < enough_convergence:
            break
        # YIELD HERE
    # RETURN HERE

def run_scenario(run_name, scene, meta_args):
    makedirs("data/%s/%d_train/" % (run_name, scene.id))
    Evaluation.data_dir = "%s/%d_train" % (run_name, scene.id)
    PSO(scene, meta_args)
    Evaluation.i = 0
    Particle.total_id = 0

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
            args.name,
            Scene(scene, args.seed),
            MetaParameters(
                args.n_particles,
                args.w_inertia,
                args.w_local,
                args.w_global
            )
        )
    # JOIN HERE
