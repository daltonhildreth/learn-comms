#Globally Optimize by using covariance matrix adaption:
#
from random import uniform, random
import numpy as np

N_PARTICLES = 20
M_SHAPE = (3,5)

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

particles = [None] * N_PARTICLES
Mg = Evaluation(np.zeros(shape=M_SHAPE))
Blo = -np.ones(shape=M_SHAPE)
Bhi = np.ones(shape=M_SHAPE)

for p in particles:
    p = Particle(Blo, Bhi)
    if p.Mp < Mg:
        Mg = p.Mp

W_INERTIA = 0.2
W_LOCAL = 0.2
W_GLOBAL = 0.2

convergent = False
enough_iter = False
while not (convergent or enough_iter):
    for p in particles:
        for row in range(M_SHAPE[0]):
            for col in range(M_SHAPE[1]):
                rp = random()
                rg = random()

                vd = p.Mv[row][col]
                pd = p.Mp.M[row][col]
                gd = Mg.M[row][col]
                xd = p.Mx.M[row][col]
                p.Mv[row][col] = W_INERTIA * vd + W_LOCAL * rp * (pd - xd) +
                    W_GLOBAL * rg * (gd - xd)
        p.Mx.M += p.Mv
        p.Mx.result = simulate(p.Mx.M)
        if p.Mx < p.Mp:
            p.Mp = p.Mx
            if p.Mp < Mg:
                Mg = p.Mp

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
