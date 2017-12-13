from random import random
from shutil import copyfile
from random import random
import os
import numpy as np

best = 0
#start = float(input("start T:"))
#schedule = float(input("schedule T:"))
epochs - int(input("enter epoch count"))
i = 0

def acceptance(e_best, e_random, T):
    if e_random < e_best:
        return 1
    else:
        return exp(-(e_best - e_random) / T)

#will get M matrix
def read_config(file):
    dims = file.readline().split(' ')
    M = np.matrix([[0 for _ in dims[0]] for _ in dims[1]])
    for i in range(dims[0]):
        line = file.readline()
        M[i] = [float(s) for s in line.split(' ')]
    return M

def write_config(file, config):
    shape = np.shape(config)
    file.write(shape[0] + " " + shape[1] + "\n")
    for i in range(0, shape[0]):
        for j in range(0, shape[1]):
            file.write(str(config[i][j]))
            if (j != shape[1] - 1)
                file.write(' ')
        file.write('\n')

#will get results to influence nn choice from M
def read_result(file):
    avg_vel = float(file.readline().strip())
    total_time = float(file.readline().strip())
    #avg_time =
    return (avg_vel, total_time)

if __name__ = '__main__':
    seed = input("seed: ")
    T = float(input("temp: "))
    rate = float(input("rate: "))
    perturb = float(input("perturb: "))

    # get baseline
    os.system("./engine/build/nocommnorender/bin/gg-engine "+seed)

    ###
    M = np.matrix([[0 for i in range(5)] for j in range(3)])

    ###
    i = 0
    i_best = -1
    best = float('inf')
    # repeat
    while (T > 0):
        # take M
        i += 1
        #---noop
        # perturb it in a random dimension
        dim = randint(0, 14)
        perturb = random(-perturb, perturb)
        new_M = deepcopy(M)
        new_M[dim/3][dim%3] += perturb

        # write M
        with config_f as open('data/comms.config'):
            write_config(config_f, new_M)

        # run sim
        os.system("./engine/build/commnorender/bin/gg-engine "+seed)

        # copy comms.result to comms_ti.result
        copyfile("data/comms.result","data/comms_t"+str(i)+".result")

        # result = avg time + 3 * std deviation
        with result_f as open('data/comms.result','r'):
            result = read_result(result_f)
            result /= baseline

        # if better, take the new M
        # otherwise, take it with probability exp(-(current - new)/T)
        if acceptance(best, result, T):
            M[dim/3][dim%3] += perturb
            best = result
            i_best = i

        # write out to comms.config
        with config_clone as open('data/comms.config','w'):
            write_config(config_clone, config)

        # copy to comms_ti.config
        copyfile('data/comms.config','data/comms_t'+str(i)+".result")

        # update temp
        T = T / (1 + rate)

    # print (order of) best
    print(M)
    print(best)
    print(i_best)
