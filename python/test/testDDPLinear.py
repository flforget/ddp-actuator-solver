__author__ = 'fforget'

import numpy as np
from DDPSolver import *
from costFunction import *
from dynamicModel import *
import matplotlib.pyplot as pl
from random import uniform
import time


Xinit = np.matrix([ [0.0],
                    [0.0],
                    [0.0]])
# Xinit = np.matrix([ [uniform(-10,10)],
#                     [uniform(-10,10)],
#                     [uniform(-10,10)]])
Xdes = np.matrix([  [0.0],
                    [0.0],
                    [1.0]])
XList = list()
UList = list()
XListtmp = list()
UListtmp = list()
timeList = list()

N = 20
dt = 1e-4

"""Debug"""
traj = 0
if (traj):
    M = 5
else:
    M = 1
trajList = [Xdes,-2*Xdes,-0.5*Xdes,-4*Xdes,2*Xdes]

model = MCCmodel()
costFunction = CostFunctionMCC3States()

solver = DDPSolver(model,costFunction)
for i in range(M):
    Xdes = trajList[i]
    initTime = time.time()
    XListtmp,UListtmp = solver.solveTrajectory(Xinit,Xdes,N,dt,20,1e-3)
    endTime = time.time() - initTime
    timeList.append(endTime/N)
    XList += XListtmp
    UList += UListtmp
    Xinit = XListtmp[N]
# for j in range(N):
#     XList.append(model.computeNextState(dt,X,U))
#     X = model.computeNextState(dt,X,U)
#     UList.append(U)

print timeList

thetaList = list()
omegaList = list()
tauList = list()
U1List = list()

for i in range((N+1)*M):
    X = XList[i]
    thetaList.append(X[0].item())
    omegaList.append(X[1].item())
    tauList.append(X[2].item())
for i in range(N*M):
    U = UList[i]
    U1List.append(U[0].item())

fig0 = pl.figure ()
ax0 = fig0.add_subplot ('221')
ax0.set_title('theta ')
ax0.plot ( range((N+1)*M),thetaList)
bx0 = fig0.add_subplot ('222')
bx0.set_title('omega')
bx0.plot ( range((N+1)*M),omegaList)
cx0 = fig0.add_subplot ('223')
cx0.set_title('tau')
cx0.plot ( range((N+1)*M),tauList)
dx0 = fig0.add_subplot ('224')
dx0.set_title('commande')
dx0.plot(range(M*N), U1List)

ax0.grid()
bx0.grid()
cx0.grid()
dx0.grid()



print costFunction.computeCostValue(N,XList,Xdes,UList)

print UList


# '''Debug'''
# Xinit = np.matrix([ [0.0],
#                     [0.0],
#                     [0.0]])
# bestCost = 1e50
# delta = 1
# for a in range(0*delta,10*delta,1):
#     for b in range(0*delta,10*delta,1):
#         for c in range(0*delta,10*delta,1):
#             UList = [a/delta,b/delta,c/delta]
#             XList = list(range(N+1))
#             XList[0] = Xinit
#             for i in range(N):
#                 XList[i+1] = model.computeNextState(dt,XList[i],UList[i])
#             currentCost = costFunction.computeCostValue(N,XList,Xdes,UList)
#             if(currentCost<bestCost):
#                 bestCost = currentCost
#                 finalUList = UList
#                 finalXList = XList
#
# print bestCost
#
# print finalUList
#
#
# thetaList = list()
# omegaList = list()
# tauList = list()
# U1List = list()
# for i in range((N+1)*M):
#     X = finalXList[i]
#     thetaList.append(X[0].item())
#     omegaList.append(X[1].item())
#     tauList.append(X[2].item())
# for i in range(N*M):
#     U = finalUList[i]
#     U1List.append(U)
#
# fig0 = pl.figure ()
# ax0 = fig0.add_subplot ('221')
# ax0.set_title('theta ')
# ax0.plot ( range((N+1)*M),thetaList)
# bx0 = fig0.add_subplot ('222')
# bx0.set_title('omega')
# bx0.plot ( range((N+1)*M),omegaList)
# cx0 = fig0.add_subplot ('223')
# cx0.set_title('tau')
# cx0.plot ( range((N+1)*M),tauList)
# dx0 = fig0.add_subplot ('224')
# dx0.set_title('commande')
# dx0.plot(range(M*N), U1List)
#
# ax0.grid()
# bx0.grid()
# cx0.grid()
# dx0.grid()


pl.show()