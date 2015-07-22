__author__ = 'fforget'

__author__ = 'fforget'

import numpy as np
from matplotlib import pyplot as pl
from dynamicModel import *
from costFunction import *
from scipy.integrate import odeint

model = SimpleRomeoActuatorDynamicModel()
cost = CostFunction()

dt = 1e-4
X = np.matrix([[0.0,0.0,0.0,0.0]]).T
Xdes = np.matrix([[1.0,0.0,0.0,0.0]]).T
U = 0.0

cost0 = list()
analyticList0 = list()
analyticList1 = list()
analyticList2 = list()
analyticList3 = list()
analyticList4 = list()
analyticList0o2 = list()
analyticList1o2 = list()
analyticList2o2 = list()
analyticList3o2 = list()
analyticList4o2 = list()
diffList0 = list()
diffList1 = list()
diffList2 = list()
diffList3 = list()
diffList4 = list()
diffList0o2 = list()
diffList1o2 = list()
diffList2o2 = list()
diffList3o2 = list()
diffList4o2 = list()
err0 = list()
err1 = list()
err2 = list()
err3 = list()
err4 = list()

nextCost = 0.0

N = 200
delta = 50.0
for i in range(N):
    X = np.matrix([ [0.0],
                    [0.0],
                    [i/delta],
                    [0.0]])
    #Xdes = X
    U = np.matrix([0.0])
    lx,lxx,lu,luu,lux,lxu= cost.computeAllCostDeriv(X,Xdes,U)
    lastCost = nextCost
    nextCost = cost.computeCostValue(X,Xdes,U)

    if((i!=0) & (i!= N-1)):
        cost0.append(cost.computeCostValue(X,Xdes,U))
        analyticList0.append(lx[0].item())
        analyticList1.append(lx[1].item())
        analyticList2.append(lx[2].item())
        analyticList3.append(lx[3].item())
        analyticList4.append(lu.item())
        analyticList0o2.append(lxx[0,0].item())
        analyticList1o2.append(lxx[1,1].item())
        analyticList2o2.append(lxx[2,2].item())
        analyticList3o2.append(lxx[3,3].item())
        analyticList4o2.append(lu.item())
        diffList0.append((nextCost-lastCost)*(delta))
        diffList1.append((nextCost-lastCost)*(delta))
        diffList2.append((nextCost-lastCost)*(delta))
        diffList3.append((nextCost-lastCost)*(delta))
        diffList4.append((nextCost-lastCost)*(delta))

        err0.append(lx[0].item() - (nextCost-lastCost)*(delta))
        err1.append(lx[1].item() - (nextCost-lastCost)*(delta))
        err2.append(lx[2].item() - (nextCost-lastCost)*(delta))
        err3.append(lx[3].item() - (nextCost-lastCost)*(delta))
        err4.append(lu.item() - (nextCost-lastCost)*(delta))
for i in range(N-3):
    diffList0o2.append((diffList0[i]- diffList0[i+1])*delta)
    diffList1o2.append((diffList1[i]- diffList1[i+1])*delta)
    diffList2o2.append((diffList2[i]- diffList2[i+1])*delta)
    diffList3o2.append((diffList3[i]- diffList3[i+1])*delta)
    diffList4o2.append((diffList4[i]- diffList4[i+1])*delta)

print analyticList0
print diffList0
print cost0

# fig0 = pl.figure ()
# ax0 = fig0.add_subplot ('221')
# ax0.set_title('tau ')
# ax0.plot ( range(N-2),analyticList0,
#           range(N-2),diffList0,'r')
# bx0 = fig0.add_subplot ('222')
# bx0.set_title('tauDot')
# bx0.plot ( range(N-2),analyticList1,
#           range(N-2),diffList1,'r')
# cx0 = fig0.add_subplot ('223')
# cx0.set_title('q')
# cx0.plot ( range(N-2),analyticList2,
#           range(N-2),diffList2,'r')
# dx0 = fig0.add_subplot ('224')
# dx0.set_title('qDot')
# dx0.plot ( range(N-2),analyticList3,
#           range(N-2),diffList3,'r')
# ax0.grid()
# bx0.grid()
# cx0.grid()
# dx0.grid()
#
# fig1 = pl.figure()
# ax1 = fig1.add_subplot ('221')
# ax1.set_title('tau ')
# ax1.plot ( range(N-2),err0)
# bx1 = fig1.add_subplot ('222')
# bx1.set_title('tauDot')
# bx1.plot ( range(N-2),err1)
# cx1 = fig1.add_subplot ('223')
# cx1.set_title('q')
# cx1.plot ( range(N-2),err2)
# dx1 = fig1.add_subplot ('224')
# dx1.set_title('qDot')
# dx1.plot ( range(N-2),err3)
# ax1.grid()
# bx1.grid()
# cx1.grid()
# dx1.grid()
#
# fig2 = pl.figure()
# ax2 = fig2.add_subplot('121')
# ax2.plot(range(N-2),analyticList4,
#          range(N-2),diffList4)
# bx2 = fig2.add_subplot('122')
# bx2.plot(range(N-2),err4)
# ax2.grid()
# bx2.grid()
#
# fig3 = pl.figure()
# ax3 = fig3.add_subplot('111')
# ax3.plot(range(N-2),cost0)
# ax3.grid()

#second order
fig4 = pl.figure()
ax4 = fig4.add_subplot('221')
ax4.plot(range(N-3),diffList0o2,
            range(N-2),analyticList0o2,'r')
bx4 = fig4.add_subplot('222')
bx4.plot(range(N-3),diffList1o2,
            range(N-2),analyticList1o2,'r')
cx4 = fig4.add_subplot('223')
cx4.plot(range(N-3),diffList2o2,
            range(N-2),analyticList2o2,'r')
dx4 = fig4.add_subplot('224')
dx4.plot(range(N-3),diffList3o2,
            range(N-2),analyticList3o2,'r')
ax4.grid()
bx4.grid()
cx4.grid()
dx4.grid()

fig5 = pl.figure()
ax5 = fig5.add_subplot('111')
ax5.plot(range(N-3),diffList4o2,
        range(N-2),analyticList4o2)
ax5.grid()

pl.show()