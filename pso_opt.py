from random import uniform, random
from shutil import copyfile
from copy import copy
from os import listdir, path, mkdir, makedirs, getpid
from os.path import isdir
from subprocess import Popen, PIPE
from itertools import chain
import argparse
import numpy as np
import asyncio

M_SHAPE = (4, 7)
N_COMM = 2


def write_config(file, config):
    file.write(str(M_SHAPE[0]) + " " + str(M_SHAPE[1]) + "\n")
    for i in range(0, M_SHAPE[0]):
        for j in range(0, M_SHAPE[1]):
            file.write(str(config[i][j]))
            if j != M_SHAPE[1] - 1:
                file.write(" ")
        file.write("\n")


def read_result(result_file):
    def read_float(file):
        return float(file.readline().strip())

    def aggregate(result):
        # this limits the results to just Julio's time confidence metric
        _, confident_time = result
        return confident_time

    with open(result_file, "r") as f:
        # will get results to influence nn choice from M
        avg_vel = read_float(f)
        confident_time = read_float(f)
        return aggregate((avg_vel, confident_time))


def pretty_mat(np_m):
    s = ""
    for r in np_m:
        for c in r:
            s += " %+.5f" % c
        s += "\n"
    return s


class Particle:
    def __init__(self, id_, min_config, max_config, scene, data_dir):
        self.id = id_
        # the baseline always gets 0
        self.iters = 1
        self.scene = scene
        self.data_dir = data_dir
        # intentionally invalid, will be replaced at next minimize with self.pos
        self.local_min = Point()

        self.pos = Point(np.zeros(shape=M_SHAPE))
        for r in range(len(self.pos.config)):
            for c in range(len(self.pos.config[r])):
                # Don't fill in values that are redundant to TTC
                if r > N_COMM - 1 and c > N_COMM - 1:
                    continue
                self.pos.config[r][c] = uniform(
                    min_config[r][c], max_config[r][c]
                )

        self.vel = np.zeros(shape=M_SHAPE)
        for r in range(len(self.vel)):
            for c in range(len(self.vel[r])):
                # Don't fill in values that are redundant to TTC
                if r > N_COMM - 1 and c > N_COMM - 1:
                    continue
                mag = abs(max_config[r][c] - min_config[r][c])
                self.vel[r][c] = uniform(-mag, mag)

    async def kickstart(self):
        vel = self.vel
        self.vel = np.zeros(shape=M_SHAPE)
        await self.attempt_move()
        self.vel = vel

    def debug(self, trace, base, global_min):
        trace.write("\t\ti %s\n" % (self.iters - 1))
        trace.write("\t\tparticle id %s\n" % self.id)
        trace.write("\t\tprocess id %d\n" % getpid())
        trace.write("xr %f\n" % self.pos.score)
        trace.write("norm xr/b %f\n" % (self.pos.score / base.score))
        trace.write("Gr %f\n" % global_min.score)
        trace.write("norm xr/Gr %f\n" % (self.pos.score / global_min.score))
        trace.write("xM\n%s\n" % pretty_mat(self.pos.config))
        trace.write("vM\n%s\n" % pretty_mat(self.vel))
        trace.write("gM\n%s\n" % pretty_mat(global_min.config))

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

    async def attempt_move(self):
        self.pos = await simulate(
            self.scene,
            self.vel + self.pos.config,
            self.data_dir,
            "%d_p%d" % (self.iters, self.id),
        )
        self.iters += 1

    def minimize(self, global_min):
        if self.pos.score < self.local_min.score:
            self.local_min = self.pos
            if self.local_min.score < global_min.score:
                global_min = self.local_min
        return global_min


class Point:
    def __init__(self, x=None, y=float("inf"), file=None):
        self.config = x
        self.score = y
        self.file = file


class Scene:
    def __init__(self, id_, seed):
        self.id = id_
        self.seed = seed


class HyperParameters:
    def __init__(
        self,
        n_particles,
        w_inertia,
        w_local,
        w_global,
        max_iters,
        max_convergence,
    ):
        self.n_particles = n_particles
        self.w_inertia = w_inertia
        self.w_local = w_local
        self.w_global = w_global
        self.max_iters = max_iters
        self.max_convergence = max_convergence


async def simulate(scene, config, data_dir, save_name):
    prog = "build/bin/comm_norender"
    # the 0 does not matter since we are using a _norender build
    data_dir += "iters/"
    args = [
        str(i) for i in [scene.id, "--seed", scene.seed, "--data", data_dir]
    ]
    data_dir = "data/" + data_dir
    save = (data_dir, save_name, save_name)

    makedirs("%s/%s/" % (data_dir, save_name))
    sim_config = "%s/comms.config" % data_dir
    with open(sim_config, "w") as config_file:
        write_config(config_file, config)
    copyfile(sim_config, "%s/%s/%s.config" % save)

    print(" ".join([prog] + args))
    with open("%s/%s/%s.log" % save, "w") as logout_file:
        with open("%s/%s/%s.err" % save, "w") as logerr_file:
            sim = await asyncio.create_subprocess_exec(
                *([prog] + args), stdout=logout_file, stderr=logerr_file
            )
            await sim.wait()

    sim_result = "%s/comms.result" % data_dir
    copyfile(sim_result, "%s/%s/%s.result" % save)
    return Point(config, read_result(sim_result), save_name)


async def PSO(data_dir, scene, hypers):
    base_scn = copy(scene)
    baseline = await simulate(
        base_scn, np.zeros(shape=M_SHAPE), data_dir, str(0)
    )
    print("baseline ==== ", baseline.score)
    global_min = Point()

    # initialize particles
    min_config = -np.ones(shape=M_SHAPE)
    max_config = np.ones(shape=M_SHAPE)
    particles = []
    for p in range(hypers.n_particles):
        p = Particle(p, min_config, max_config, scene, data_dir)
        await p.kickstart()
        global_min = p.minimize(global_min)
        with open("data/%s/opt.log" % data_dir, "a") as trace:
            p.debug(trace, baseline, global_min)
        particles += [p]

    conv_path = data_dir + "/convergence.log"
    with open("data/" + conv_path, "w") as f_conv:
        f_conv.write("convergence value per full iteration of all particles\n")

    for i in range(hypers.max_iters):
        for p in particles:
            # integrate particle swarm dynamics
            p.gravitate(global_min, hypers)
            await p.attempt_move()

            # update known minima
            global_min = p.minimize(global_min)
            with open("data/%s/opt.log" % data_dir, "a") as trace:
                p.debug(trace, baseline, global_min)

        # the potential variation of the fastest particle
        convergence = abs(max([np.linalg.norm(p.vel) for p in particles]))
        with open("data/" + conv_path, "a") as f_conv:
            f_conv.write("i: " + str(i) + ",  c: " + str(convergence) + "\n")
        if convergence < hypers.max_convergence:
            return "conv", baseline.score, global_min.score, global_min.file
    return "iter", baseline.score, global_min.score, global_min.file


def gen_result(run, scene, minimum):
    how, base_val, min_val, min_file = minimum
    src = "data/%s/%d_train/iters" % (run, scene.id)
    dst = "data/%s/results/%d_min" % (run, scene.id)
    makedirs(dst)
    src = (src, min_file, min_file)
    copyfile("%s/%s/%s.config" % src, "%s/comms.config" % dst)
    copyfile("%s/%s/%s.result" % src, "%s/%s.result" % (dst, min_file))

    # simulate_with_video(scene, np.zeroes(...), other_stuff_not_needed )
    # simulate_with_video(scene, min_file.config, other_stuff_not_needed )

    # ffmpeg -i ttc.mp4 -i comm.mp4 -filter_complex hstack split.mp4

    return (
        str(scene.id)
        + ("\tbest at config: " + min_file)
        + ("\tw/ conf_time: " + str(min_val))
        + ("\tvs base_conf_time: " + str(base_val))
        + ("\tby " + how)
        + "\n"
    )


def agg_result(run, per_result_texts):
    with open("data/%s/results/best.txt" % run, "w") as best:
        for text in per_result_texts:
            best.write(text)


async def run_scenario(run_name, scene, meta_args):
    data_dir = "%s/%d_train/" % (run_name, scene.id)
    makedirs("data/" + data_dir)
    minimum = await PSO(data_dir, scene, meta_args)
    return gen_result(run_name, scene, minimum)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="pso_opt",
        description="Globally optimize via Particle-Swarms the communication"
        "model for the Talking Hive project.",
    )
    parser.add_argument("name", type=str)
    parser.add_argument("--n_particles", type=int, default=12)
    parser.add_argument("--w_inertia", type=float, default=0.2)
    parser.add_argument("--w_local", type=float, default=0.2)
    parser.add_argument("--w_global", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=511_607_575)
    parser.add_argument("--iters", type=int, default=100)
    parser.add_argument("--convergence", type=float, default=1e-9)
    args = parser.parse_args()

    # For a high-Level of what should be going on, see test_async.py
    event_loop = asyncio.get_event_loop()
    try:
        scenes = list(chain(range(0, 10 + 1), range(14, 17 + 1)))
        tasks = (
            run_scenario(
                args.name,
                Scene(scene, args.seed),
                HyperParameters(
                    args.n_particles,
                    args.w_inertia,
                    args.w_local,
                    args.w_global,
                    args.iters,
                    args.convergence,
                ),
            )
            for scene in scenes
        )
        optimize = asyncio.gather(*tasks, return_exceptions=True)
        results = event_loop.run_until_complete(optimize)
        agg_result(args.name, results)
        for i, outcome in enumerate(optimize.result()):
            if isinstance(outcome, Exception):
                print("ERROR", i, scenes[i], outcome)
            else:
                print(i, scenes[i], outcome)

    finally:
        event_loop.close()
