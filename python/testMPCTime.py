__author__ = 'fforget'

import numpy as np
from iLQRSolver import *
from iLQRSolverImproved import *
from costFunction import *
from costFunctionImproved import *
from dynamicModel import *
from dynamicModelImproved import *
import matplotlib.pyplot as pl
from random import uniform
import time


Xinit = np.matrix([ [0.0],
                    [0.0],
                    [0.0],
                    [0.0]])
Xinit = np.matrix([ [uniform(-10,10)],
                    [uniform(-0.1,0.1)],
                    [uniform(-10,10)],
                    [uniform(-0.1,0.1)]])
Xdes = np.matrix([  [1.0],
                    [0.0],
                    [0.0],
                    [0.0]])
XList = list()
UList = list()
XListtmp = list()
UListtmp = list()
timeList = list()
timeList1 = list()

T = 20 # preview steps
N = 40 # total steps

dt = 1e-4

model = SimpleRomeoActuatorDynamicModel()
modelImproved = SimpleRomeoActuatorDynamicModelImproved()
costFunction = CostFunctionSimpleRomeoActuator()
costFunctionImproved = CostFunctionSimpleRomeoActuatorImproved()


solver = ILQRSolver(model,costFunction)
solverImproved = ILQRSolverImproved(modelImproved,costFunctionImproved)

XListList = list()
UListList = list()

finalXList = list()
finalUList = list()

finalXList.append(Xinit)
for i in range(N):
    t1 = time.time()
    XList,UList = solver.solveTrajectory(Xinit,Xdes,T,dt,20,1e-3)
    t2 = time.time()
    timeList.append((t2-t1)/T)
    Xinit = solver.model.computeNextState(dt,Xinit,UList[0])

for i in range(N):
    t1 = time.time()
    XList,UList = solver.solveTrajectory(Xinit,Xdes,T,dt,20,1e-3)
    t2 = time.time()
    timeList1.append((t2-t1)/T)
    Xinit = solver.model.computeNextState(dt,Xinit,UList[0])


print timeList
print timeList1

pl.plot(timeList)
pl.plot(timeList1,'r')


pl.show()
