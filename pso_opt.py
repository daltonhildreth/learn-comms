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

N_COMM = 3
# FIXME: Change this TO +7 if NORM_REL is defined
M_SHAPE = (N_COMM+2, N_COMM+5)


def write_config(file, config):
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
        if np.isnan(confident_time) or confident_time > 1e99:
            confident_time = 1e99
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
    def __init__(
        self, id_, min_config, max_config, scene_set, data_dir, baselines
    ):
        self.id = id_
        # the baseline always gets 0
        self.iters = 1
        self.scene_set = scene_set
        self.data_dir = data_dir
        # intentionally invalid, will be replaced at next minimize with self.pos
        self.local_min = Point()
        self.baselines = baselines

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

    async def kickstart(self, build_dir):
        vel = self.vel
        self.vel = np.zeros(shape=M_SHAPE)
        await self.attempt_move(build_dir)
        self.vel = vel

    def debug(self, trace, base, global_min):
        trace.write("\t\ti %s\n" % (self.iters - 1))
        trace.write("\t\tparticle id %s\n" % self.id)
        trace.write("\t\tprocess id %d\n" % getpid())
        trace.write("xr %f\n" % (self.pos.score * base.score))
        trace.write("norm xr/b %f\n" % (self.pos.score))
        trace.write("Gf %s\n" % global_min.file)
        trace.write("Gr %f\n" % (global_min.score * base.score))
        trace.write("norm Gr/b %f\n" % global_min.score)
        trace.write("xr/Gr %f\n" % (self.pos.score / global_min.score))
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

    async def attempt_move(self, build_dir):
        self.pos = await simulate_set(
            self.scene_set,
            self.vel + self.pos.config,
            self.data_dir,
            "%d_p%d" % (self.iters, self.id),
            lambda res: sum([r.norm(self.baselines[i]) for i, r in enumerate(res)]),
            build_dir,
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

    def norm(self, baseline):
        return Point(self.config, self.score / baseline.score, self.file)

    def __add__(self, other):
        assert self.file == other.file
        return Point(self.config, self.score + other.score, self.file)

    def __radd__(self, sum_score):
        return Point(self.config, self.score + sum_score, self.file)


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
        build_dir,
    ):
        self.n_particles = n_particles
        self.w_inertia = w_inertia
        self.w_local = w_local
        self.w_global = w_global
        self.max_iters = max_iters
        self.max_convergence = max_convergence
        self.build_dir = build_dir


async def log_exec(prog, args, where):
    print(" ".join([prog] + args))
    with open("%s.out" % where, "w") as stdout:
        with open("%s.err" % where, "w") as stderr:
            p = await asyncio.create_subprocess_exec(
                *([prog] + args), stdout=stdout, stderr=stderr
            )
            await p.wait()


async def simulate_set(scene_set, config, data_dir, save_name, agg, build_dir):
    sims = (
        simulate(scene, config, "%s/s%d_" % (data_dir, scene.id), save_name, build_dir)
        for scene in scene_set
    )
    batch = await asyncio.gather(*sims, return_exceptions=True)
    return agg(batch)


async def simulate(scene, config, data_dir, save_name, build_dir):
    data_dir += "iters/"
    prog = "%s/bin/comm_norender" % build_dir
    args = [scene.id, "--seed", scene.seed, "--data", data_dir]
    args = [str(i) for i in args]
    data_dir = "data/" + data_dir
    save = (data_dir, save_name, save_name)

    # write config to data_dir/comms.config and data_dir/save_name/save_name.config
    makedirs("%s/%s/" % (data_dir, save_name))
    sim_config = "%s/comms.config" % data_dir
    with open(sim_config, "w") as config_file:
        write_config(config_file, config)
    copyfile(sim_config, "%s/%s/%s.config" % save)

    # run comm_norender with scene and data_dir/comms.config
    await log_exec(prog, args, "%s/%s/%s" % save)

    # copy exec's result to data_dir/save_name/save_name.result
    sim_result = "%s/comms.result" % data_dir
    copyfile(sim_result, "%s/%s/%s.result" % save)

    return Point(config, read_result(sim_result), save_name)


SEMAPHORE = None


async def record(scene, data_dir, with_comm, build_dir):
    prog = ("%s/bin/" % build_dir) + ("" if with_comm else "no") + "comm_render"
    args = [scene.id, "--seed", scene.seed, "--data", data_dir, "--record", 0]
    args = [str(i) for i in args]
    data_dir = "data/" + data_dir

    try:
        async with SEMAPHORE:
            await log_exec(prog, args, "%s/%d_%d" % (data_dir, scene.id, with_comm))
    except RuntimeError:
        pass


async def hstack(left, right, out):
    args = ["-i", left, "-i", right, "-filter_complex", "hstack", out]
    args = [str(i) for i in args]
    await log_exec("ffmpeg", args, out)


async def validate(run, model_id, scene, build_dir):
    src = "%s/t%d_min/" % (run, model_id)
    dst = src + "v%d" % scene.id
    makedirs("data/" + dst)
    copyfile("data/%s/comms.config" % src, "data/%s/comms.config" % dst)

    await record(scene, dst, False, build_dir)
    b_result = read_result("data/%s/comms.result" % dst)
    copyfile("data/%s/agent_paths.csv" % dst, "data/%s/b_agent_paths.csv" % dst)
    await record(scene, dst, True, build_dir)
    result = read_result("data/%s/comms.result" % dst)
    dst = "data/" + dst
    await hstack("%s/ttc.mp4" % dst, "%s/comm.mp4" % dst, "%s/split.mp4" % dst)

    return (scene.id, result, b_result)


async def PSO(data_dir, scene_set, hypers):
    baselines = await simulate_set(
        scene_set, np.zeros(shape=M_SHAPE), data_dir, str(0), list, hypers.build_dir
    )
    baseline = sum(baselines)

    # initialize particles
    global_min = Point()
    min_config = -np.ones(shape=M_SHAPE)
    max_config = np.ones(shape=M_SHAPE)
    particles = []
    for p in range(hypers.n_particles):
        print("s0=%d i=1 p=%d" % (scene_set[0].id, p), end="\t")
        p = Particle(p, min_config, max_config, scene_set, data_dir, baselines)
        await p.kickstart(hypers.build_dir)
        global_min = p.minimize(global_min)
        with open("data/%s/opt.log" % data_dir, "a") as trace:
            p.debug(trace, baseline, global_min)
        particles += [p]

    conv_path = data_dir + "/convergence.log"
    with open("data/" + conv_path, "w") as conv_f:
        conv_f.write("convergence value per full iteration of all particles\n")

    for i in range(hypers.max_iters):
        for p in particles:
            print("s0=%d i=%d p=%d" % (scene_set[0].id, i+2, p.id), end="\t")
            # integrate particle swarm dynamics
            p.gravitate(global_min, hypers)
            await p.attempt_move(hypers.build_dir)

            # update known minima
            global_min = p.minimize(global_min)
            with open("data/%s/opt.log" % data_dir, "a") as trace:
                p.debug(trace, baseline, global_min)

        # the mean of the per-dimension IQR of particle positions; "spread"
        positions = [p.pos.config for p in particles]
        q3, q1 = np.percentile(positions, [75, 25], axis=0)
        convergence = np.mean(q3 - q1)
        with open("data/" + conv_path, "a") as f_conv:
            f_conv.write("i: " + str(i) + ",  c: " + str(convergence) + "\n")
        # if convergence < hypers.max_convergence:
        #    return "conv", baseline.score, global_min.score, global_min.file
    return "iter", baseline.score, global_min.score, global_min.file


def cp_min(run, model_id, scene_set, minimum):
    _, _, _, min_particle = minimum
    # just take the first scene's as they will all have the same .config for
    # the same particle
    src = "data/%s/t%d_train/s%d_iters" % (run, model_id, scene_set[0].id)
    dst = "data/%s/t%d_min" % (run, model_id)
    makedirs(dst)
    src = (src, min_particle, min_particle)
    copyfile("%s/%s/%s.config" % src, "%s/comms.config" % dst)
    copyfile("%s/%s/%s.result" % src, "%s/%s.result" % (dst, min_particle))


def summarize(run, per_result_table, task_summary):
    with open("data/%s/summary.tsv" % run, "a") as summary:
        summary.write(task_summary)

    n_models = len(per_result_table)
    per_result_table = list(zip(*per_result_table))

    with open("data/%s/best.tsv" % run, "a") as best:
        header = ["model#", "min config", "comm Jt", "ttc Jt", "exit"]
        best.write("\t".join(header) + "\n")
        for row in per_result_table[0]:
            best.write("\t".join(row) + "\n")

    # write out summary of each name/t#_min/cross_val.tsv file
    with open("data/%s/cross_val.tsv" % run, "a") as cross_val:
        header = ["tests", *["%d\t%d/b" % (i, i) for i in range(n_models)], "b"]
        cross_val.write("\t".join(header) + "\n")

        # Determine which scenes were tested, so only they are output as rows
        # even when a model didn't test that scene
        tested_scenes = set()
        for model in per_result_table[1]:
            for scene in model:
                tested_scenes.add(scene[0])
        tested_scenes = sorted(tested_scenes)

        # build matrix of models (columns) validated on tests (rows)
        # None means that model didn't test that scene
        xcorr_matrix = []
        for _ in range(len(tested_scenes)):
            xcorr_matrix += [[None] * (n_models + 1)]
        for col, model in enumerate(per_result_table[1]):
            for row, (scene_id, result, baseline) in enumerate(model):
                xcorr_scene = tested_scenes.index(scene_id)
                xcorr_matrix[xcorr_scene][col] = result
                if not xcorr_matrix[xcorr_scene][-1]:
                    xcorr_matrix[xcorr_scene][-1] = baseline

        # write out the rows of the xcorr_matrix as lines. None -> "--------"
        # each result becomes the result & result / baseline
        # only 1 baseline is written at the end of the line
        for scene, row in enumerate(xcorr_matrix):
            baseline = xcorr_matrix[scene][-1]
            line = [str(tested_scenes[scene])]
            for model, result in enumerate(row[:-1]):
                if result == None:
                    line += ["--------"] * 2
                else:
                    line += ["%f" % result, "%f" % (result / baseline)]
            cross_val.write("\t".join(line + ["%f" % baseline]) + "\n")


async def train(run_name, model_id, scene_set, meta_args):
    data_dir = "%s/t%d_train" % (run_name, model_id)
    makedirs("data/" + data_dir)
    minimum = await PSO(data_dir, scene_set, meta_args)
    cp_min(run_name, model_id, scene_set, minimum)
    how, base_val, min_val, min_particle = minimum
    ret = [str(model_id), min_particle, "%f" % min_val, "%f" % base_val, how]

    with open("data/%s/t%d_min/best.tsv" % (run_name, model_id), "w") as f:
        f.write("model#\tmin p\tcomm Jt\tttc Jt\texit\n")
        f.write("\t".join(ret) + "\n")
    return ret


async def cross_validate(name, model_id, test_set, build_dir):
    tests = (validate(name, model_id, scene, build_dir) for scene in test_set)
    cross_val = await asyncio.gather(*tests)
    with open("data/%s/t%d_min/cross_val.tsv" % (name, model_id), "a") as f:
        f.write("model_id\t%d\n" % model_id)
        for i, scene in enumerate(test_set):
            scene_val = [str(j) for j in cross_val[i][1:]]
            f.write("\t".join([str(scene.id)] + scene_val) + "\n")
    return cross_val


async def task_batch(args, model_id, batch, test_sets):
    batch = [Scene(scene, args.seed) for scene in batch]
    hyper = HyperParameters(
        args.n_particles,
        args.w_inertia,
        args.w_local,
        args.w_global,
        args.iters,
        args.convergence,
        args.program
    )
    model = await train(args.name, model_id, batch, hyper)
    cross_val = await cross_validate(
        args.name,
        model_id,
        [Scene(scene, args.seed) for scene in test_sets[model_id]],
        args.program
    )
    return model, cross_val, True


async def task_reload(name, model_name, retest_sets, program):
    # expect args.name/t(model_name)_min to exist as a dir
    # use args.name/t(model_name)_min as the data_dir for recording comms.config
    # cross_validate retest_sets on the data_dir, but append to cross_val.tsv
    cross_val = await cross_validate(
        name,
        model_name,
        [Scene(scene, args.seed) for scene in retest_sets[model_name]],
        program,
    )
    return ["rl" + str(model_name)], cross_val, False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="pso_opt",
        description="Globally optimize via Particle-Swarms the communication"
        "model for the Talking Hive project.",
    )
    parser.add_argument("name", type=str)
    parser.add_argument("--n_particles", type=int, default=40)  # 30)
    parser.add_argument("--w_inertia", type=float, default=0.729)
    parser.add_argument("--w_local", type=float, default=1.494)
    parser.add_argument("--w_global", type=float, default=1.494)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--iters", type=int, default=225)  # 300) # 450)
    parser.add_argument("--convergence", type=float, default=1e-9)
    # not really str, only sometimes. Usually int... : (a or int...)
    parser.add_argument("--batch", type=str, nargs="*", action="append")
    # not really str, only sometimes. Usually int...
    parser.add_argument("--reload", type=str, nargs="*", action="append")
    parser.add_argument("--program", type=str, default="build")
    args = parser.parse_args()

    all_scenes = [0, 2, 4, 6, 8, 9, 10, 13, 15, 18]#list(range(0, 18 + 1))

    train_sets = []
    test_sets = []
    if args.batch:
        train_sets = [i[: i.index(":")] for i in args.batch]
        train_sets = [[int(i) for i in s] for s in train_sets]
        test_sets = [i[i.index(":") + 1 :] for i in args.batch]
        for i, v in enumerate(test_sets):
            if "all".find(v[0].lower()) == 0:
                test_sets[i] = all_scenes
            else:
                for j, u in enumerate(v):
                    test_sets[i][j] = int(u)

    reload_models = []
    retest_sets = []
    if args.reload:
        reload_models = [int(i[0]) for i in args.reload]
        retest_sets = [i[1:] for i in args.reload]
        for i, v in enumerate(retest_sets):
            if "all".find(v[0].lower()) == 0:
                retest_sets[i] = all_scenes
            else:
                for j, u in enumerate(v):
                    retest_sets[i][j] = int(u)

    # Final directory layout of run:
    # best.tsv
    # t0_train/
    #   s0_iters/
    #     comms.config
    #     comms.result
    #     0
    #     1_p0..
    #   s9_iters/
    #   ..log
    # t0_min/
    #   *_p*.result
    #   comms.config
    #   v0/...
    #   ...v17/
    #   cross_val.tsv
    # t1_train/
    #   s0_iters/
    #   ..log
    # t1_min/
    # t2_train/
    #   s9_iters/
    #   ..log
    # t2_min/

    # For a high-Level of what should be going on, see test_async.py
    event_loop = asyncio.get_event_loop()
    try:
        tasks = chain(
            (
                task_batch(args, i, batch, test_sets)
                for i, batch in enumerate(train_sets)
            ),
            (
                task_reload(args.name, model, retest_sets, args.program)
                for model in reload_models
            ),
        )

        SEMAPHORE = asyncio.Semaphore(len(all_scenes))
        optimize = asyncio.gather(*tasks)  # , return_exceptions=True)
        minima = event_loop.run_until_complete(optimize)
        task_summary = "model  sets  outcome\n"
        for i, outcome in enumerate(optimize.result()):
            if isinstance(outcome, Exception):
                task_summary += str(train_sets) + "\n"
                task_summary += str("ERROR") + str(i) + str(outcome) + "\n"
            else:
                # is batch or reload
                if outcome[2]:
                    task_summary += str(i) + str(train_sets[i]) + "\n"
                else:
                    task_summary += str(i) + str(reload_models[i]) + "\n"
                # model info (#, config, L(c), ???(b), exit how?) or (rl#)
                task_summary += str("\t") + str(outcome[0]) + "\n"

                a = [0, 0, 0]
                # for result in cross_val
                for j in outcome[1]:
                    task_summary += str("\t") + str(j[:]) + str(j[1]/j[2]) + "\n"
                    a[0] += j[1]
                    a[1] += j[2]
                    a[2] += j[1]/j[2]
                task_summary += str("\tsums:") + str(a) + "\n"
                task_summary += str("\t^/^") + str(a[0]/a[1]) + "\n"
                task_summary += str("\t    ^/n") + str(a[2]/len(outcome[1])) + "\n"
        print(task_summary)
        summarize(args.name, minima, task_summary)

    finally:
        event_loop.close()
