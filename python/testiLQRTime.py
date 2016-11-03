__author__ = 'fforget'

import numpy as np
from iLQRSolverImproved import *
from iLQRSolver import *
from costFunction import *
from dynamicModel import *
from costFunctionImproved import *
from dynamicModelImproved import *
import matplotlib.pyplot as pl
from random import uniform
import time

Xinit = np.matrix([ [uniform(-10,10)],
                    [uniform(-1.0,1.0)],
                    [uniform(-10,10)],
                    [uniform(-1.0,1.0)]])
Xdes = np.matrix([  [uniform(-10,10)],
                    [0.0],
                    [0.0],
                    [0.0]])

XList = list()
UList = list()
timeList = list()
timeImprovedList = list()

N = 20
dt = 1e-4


model = SimpleRomeoActuatorDynamicModel()
costFunction = CostFunctionSimpleRomeoActuator()
modelImproved = SimpleRomeoActuatorDynamicModelImproved()
costFunctionImproved = CostFunctionSimpleRomeoActuatorImproved()


solver = ILQRSolver(model,costFunction)
solverImproved = ILQRSolverImproved(modelImproved,costFunctionImproved)


for N in range(1,30,1):

    Xinit = np.matrix([ [uniform(-10,10)],
                    [uniform(-1.0,1.0)],
                    [uniform(-10,10)],
                    [uniform(-1.0,1.0)]])
    Xdes = np.matrix([  [uniform(-10,10)],
                    [0.0],
                    [0.0],
                    [0.0]])
    initTime = time.time()
    for i in range(5):
        solver.solveTrajectory(Xinit,Xdes,N,dt,20,1e-3)
    endTime = time.time() - initTime
    timeList.append(endTime/(5*N))


    Xinit = np.matrix([ [uniform(-10,10)],
                    [uniform(-1.0,1.0)],
                    [uniform(-10,10)],
                    [uniform(-1.0,1.0)]])
    Xdes = np.matrix([  [uniform(-10,10)],
                    [0.0],
                    [0.0],
                    [0.0]])
    initTime = time.time()
    for i in range(5):
        solverImproved.solveTrajectory(Xinit,Xdes,N,dt,20,1e-3)
    endTime = time.time() - initTime
    timeImprovedList.append(endTime/(5*N))

print timeList
print timeImprovedList

pl.plot(timeList)
pl.plot(timeImprovedList,'r')
pl.grid()


pl.show()