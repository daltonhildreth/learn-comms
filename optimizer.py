from random import random
from shutil import copyfile
from random import random, randint, uniform
from copy import copy
from math import exp
import os
import numpy as np

best = 0
#start = float(input("start T:"))
#schedule = float(input("schedule T:"))
i = 0

def acceptance(e_best, e_random, T):
    if e_random < e_best:
        return 1
    else:
        return exp(-(e_best - e_random) / T)

def write_config(file, config):
    file.write("3 5\n")
    for i in range(0, 3):
        for j in range(0, 5):
            file.write(str(config[i*5+j]))
            if (j != 4):
                file.write(' ')
        file.write('\n')

#will get results to influence nn choice from M
def read_result(file):
    avg_vel = float(file.readline().strip())
    confident_time = float(file.readline().strip())
    #avg_time =
    return (avg_vel, confident_time)

if __name__ == '__main__':
    seed = input("seed: ")
    T = float(input("temp: "))
    rate = float(input("rate: "))
    perturb = float(input("perturb: "))

    # get baseline
    os.system("build/nocomm_norender/bin/gg-engine "+seed)
    # result = avg time + 3 * std deviation
    with open('data/comms.result','r') as result_f:
        baseline = read_result(result_f)[1]
    # copy comms.result to comms_ti.result
    copyfile("data/comms.result","data/comms_base.result")
    print("base",baseline)

    ###
    M = [0 for i in range(15)]

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
        perturb = uniform(-perturb, perturb)
        new_M = copy(M)
        new_M[dim] += perturb
        print(new_M)

        # write M
        with open('data/comms.config','w') as config_f:
            write_config(config_f, new_M)

        # run sim
        os.system("build/comm_norender/bin/gg-engine "+seed)

        # copy comms.result to comms_ti.result
        copyfile("data/comms.result","data/comms_t"+str(i)+".result")

        # result = avg time + 3 * std deviation
        with open('data/comms.result','r') as result_f:
            result = read_result(result_f)[1]
            print("r1",result)
            result /= baseline
            print("norm",result)

        # if better, take the new M
        # otherwise, take it with probability exp(-(current - new)/T)
        if acceptance(best, result, T):
            M[dim] += perturb
            best = result
            i_best = i

        # write out to comms.config
        with open('data/comms.config','w') as config_clone:
            write_config(config_clone, new_M)

        # copy to comms_ti.config
        copyfile('data/comms.config','data/comms_t'+str(i)+".config")

        # update temp
        T -= rate

    # print (order of) best
    print(M)
    print()
    print(best)
    print(i_best)
