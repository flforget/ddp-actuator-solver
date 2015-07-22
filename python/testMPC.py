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

T = 20 # preview steps
N = 50 # total steps

"""Debug"""
traj = False
if (traj):
    M = 5
else:
    M = 1
desList = list([Xdes,-2*Xdes,-0.5*Xdes,Xdes,2*Xdes])
desList = list([uniform(-10,10)*Xdes,
                uniform(-10,10)*Xdes,
                uniform(-10,10)*Xdes,
                uniform(-10,10)*Xdes,
                uniform(-10,10)*Xdes])
trajList = list()
for i in range(N*M):
    trajList.append(desList[i//N])

dt = 1e-4

model = SimpleRomeoActuatorDynamicModel()
costFunction = CostFunctionSimpleRomeoActuator()

solver = ILQRSolver(model,costFunction)

XListList = list()
UListList = list()

finalXList = list()
finalUList = list()

finalXList.append(Xinit)
for i in range(M*N):
    Xdes = trajList[i]
    initTime = time.time()
    XList,UList = solver.solveTrajectory(Xinit,Xdes,T,dt,20,1e-3)
    endTime = time.time() - initTime
    XListList.append(XList)
    UListList.append(UList)
    timeList.append(endTime)
    Xinit = solver.model.computeNextState(dt,Xinit,UList[0])
    finalXList.append(Xinit)
    finalUList.append(UList[0].item())


print timeList

finalTauList = list()
finalTauDotList = list()
finalQList = list()
finalQDotList = list()
for i in range(M*(N)+1):
    X = finalXList[i]
    finalTauList.append(X[0].item())
    finalTauDotList.append(X[1].item())
    finalQList.append(X[2].item())
    finalQDotList.append(X[3].item())

'''DEBUG'''
fig0 = pl.figure()
ax0 = fig0.add_subplot('221')
bx0 = fig0.add_subplot('222')
cx0 = fig0.add_subplot('223')
dx0 = fig0.add_subplot('224')
ax0.set_title('Tau')
bx0.set_title('TauDot')
cx0.set_title('Q')
dx0.set_title('QDot')
ax0.grid()
bx0.grid()
cx0.grid()
dx0.grid()

fig00 = pl.figure()
ax00 = fig00.add_subplot('111')
ax00.grid()
ax00.set_title('Command')

for i in range(M*N):
    currentXList = XListList[i]
    currentUList = UListList[i]
    tauList = list()
    tauDotList = list()
    qList = list()
    qDotList = list()
    U1List = list()
    for j in range(T):
        U = currentUList[j]
        X = currentXList[j]
        tauList.append(X[0].item())
        tauDotList.append(X[1].item())
        qList.append(X[2].item())
        qDotList.append(X[3].item())
        U1List.append(U[0].item())
    X = currentXList[T]
    tauList.append(X[0].item())
    tauDotList.append(X[1].item())
    qList.append(X[2].item())
    qDotList.append(X[3].item())
    ax0.plot(range(i,T+i+1,1),tauList)
    bx0.plot(range(i,T+i+1,1),tauDotList)
    cx0.plot(range(i,T+i+1,1),qList)
    dx0.plot(range(i,T+i+1,1),qDotList)
    ax00.plot(range(i,T+i,1),U1List)

ax0.plot(range(M*N+1),finalTauList)
bx0.plot(range(M*N+1),finalTauDotList)
cx0.plot(range(M*N+1),finalQList)
dx0.plot(range(M*N+1),finalQDotList)

ax00.plot(range(M*N),finalUList)



# print tauList
# print XList
# print UList
print timeList


pl.show()
