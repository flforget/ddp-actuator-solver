__author__ = 'fforget'

import numpy as np
from iLQRSolver import *
from costFunction import *
from dynamicModel import *
import matplotlib.pyplot as pl
from random import uniform
import time


Xinit = np.matrix([ [0.0],
                    [0.0],
                    [0.0],
                    [0.0]])
# Xinit = np.matrix([ [uniform(-10,10)],
#                     [uniform(-1.0,1.0)],
#                     [uniform(-10,10)],
#                     [uniform(-1.0,1.0)]])
Xdes = np.matrix([  [1.0],
                    [0.0],
                    [0.0],
                    [0.0]])

XList = list()
UList = list()
XListtmp = list()
UListtmp = list()
timeList = list()

N = 3

"""Debug"""
traj = False
if (traj):
    M = 5
else:
    M = 1
trajList = list([Xdes,-2*Xdes,-0.5*Xdes,Xdes,2*Xdes])
dt = 1e-4

model = SimpleRomeoActuatorDynamicModel()
costFunction = CostFunctionSimpleRomeoActuator()

solver = ILQRSolver(model,costFunction)
for i in range(M):
    Xdes = trajList[i]
    initTime = time.time()
    XListtmp,UListtmp = solver.solveTrajectory(Xinit,Xdes,N,dt,20,1e-3)
    endTime = time.time() - initTime
    print costFunction.computeCostValue(N,XListtmp,Xdes,UListtmp)
    timeList.append(endTime/N)
    XList += XListtmp
    UList += UListtmp
    Xinit = XListtmp[N]


tauList = list()
tauDotList = list()
qList = list()
qDotList = list()
U1List = list()
for i in range(M*(N+1)):
    X = XList[i]
    tauList.append(X[0].item())
    tauDotList.append(X[1].item())
    qList.append(X[2].item())
    qDotList.append(X[3].item())
for i in range(M*N):
    U = UList[i]
    U1List.append(U.item())

# print tauList
# print XList
# print UList
print timeList


fig0 = pl.figure ()
ax0 = fig0.add_subplot ('221')
ax0.set_title('tau ')
ax0.plot ( range(M*(N+1)),tauList)
bx0 = fig0.add_subplot ('222')
bx0.set_title('tauDot')
bx0.plot ( range(M*(N+1)),tauDotList)
cx0 = fig0.add_subplot ('223')
cx0.set_title('q')
cx0.plot ( range(M*(N+1)),qList)
dx0 = fig0.add_subplot ('224')
dx0.set_title('qDot')
dx0.plot ( range(M*(N+1)),qDotList)
ax0.grid()
bx0.grid()
cx0.grid()
dx0.grid()

fig1 = pl.figure()
ax1 = fig1.add_subplot ('111')
ax1.set_title('commande ')
ax1.plot ( range(M*N),U1List)
ax1.grid()
pl.show()