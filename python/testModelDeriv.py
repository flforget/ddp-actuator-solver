__author__ = 'fforget'

import numpy as np
from matplotlib import pyplot as pl
from dynamicModel import *
from scipy.integrate import odeint

model = simpleRomeoActuatorDynamicModel()

dt = 1e-4
X = np.matrix([[0.0,0.0,0.0,0.0]]).T
U = 0.0

analyticList0 = list()
analyticList1 = list()
analyticList2 = list()
analyticList3 = list()
diffList0 = list()
diffList1 = list()
diffList2 = list()
diffList3 = list()
err0 = list()
err1 = list()
err2 = list()
err3 = list()
analyticList0o2 = list()
analyticList1o2 = list()
analyticList2o2 = list()
analyticList3o2 = list()
diffList0o2 = list()
diffList1o2 = list()
diffList2o2 = list()
diffList3o2 = list()
err0o2 = list()
err1o2 = list()
err2o2 = list()
err3o2 = list()
analyticList0U = list()
analyticList1U = list()
analyticList2U = list()
analyticList3U = list()
diffList0U = list()
diffList1U = list()
diffList2U = list()
diffList3U = list()
err0U = list()
err1U = list()
err2U = list()
err3U = list()

nextX = X

N = 100
delta = 10000.0
for i in range(N):
    X = np.matrix([ [0.0],
                    [0.0],
                    [0.0],
                    [i/delta]])
    fx,fu,fxx,fuu,fux = model.computeAllModelDeriv(dt,X,U)
    lastX = nextX
    nextX = model.computeNextState(dt,X,U)

    if((i!=0) & (i!= N-1)):
        analyticList0.append(fx[0,3].item())
        analyticList1.append(fx[1,3].item())
        analyticList2.append(fx[2,3].item())
        analyticList3.append(fx[3,3].item())
        diffList0.append((nextX[0].item()-lastX[0].item())*(delta))
        diffList1.append((nextX[1].item()-lastX[1].item())*(delta))
        diffList2.append((nextX[2].item()-lastX[2].item())*(delta))
        diffList3.append((nextX[3].item()-lastX[3].item())*(delta))

        err0.append(fx[0,3].item() - (nextX[0].item()-lastX[0].item())*(delta))
        err1.append(fx[1,3].item() - (nextX[1].item()-lastX[1].item())*(delta))
        err2.append(fx[2,3].item() - (nextX[2].item()-lastX[2].item())*(delta))
        err3.append(fx[3,3].item() - (nextX[3].item()-lastX[3].item())*(delta))

fig0 = pl.figure ()
ax0 = fig0.add_subplot ('221')
ax0.set_title('tau ')
ax0.plot ( range(N-2),analyticList0,
          range(N-2),diffList0,'r')
bx0 = fig0.add_subplot ('222')
bx0.set_title('tauDot')
bx0.plot ( range(N-2),analyticList1,
          range(N-2),diffList1,'r')
cx0 = fig0.add_subplot ('223')
cx0.set_title('q')
cx0.plot ( range(N-2),analyticList2,
          range(N-2),diffList2,'r')
dx0 = fig0.add_subplot ('224')
dx0.set_title('qDot')
dx0.plot ( range(N-2),analyticList3,
          range(N-2),diffList3,'r')
ax0.grid()
bx0.grid()
cx0.grid()
dx0.grid()

fig1 = pl.figure()
ax1 = fig1.add_subplot ('221')
ax1.set_title('tau ')
ax1.plot ( range(N-2),err0)
bx1 = fig1.add_subplot ('222')
bx1.set_title('tauDot')
bx1.plot ( range(N-2),err1)
cx1 = fig1.add_subplot ('223')
cx1.set_title('q')
cx1.plot ( range(N-2),err2)
dx1 = fig1.add_subplot ('224')
dx1.set_title('qDot')
dx1.plot ( range(N-2),err3)
ax1.grid()
bx1.grid()
cx1.grid()
dx1.grid()


for i in range(N):
    X = np.matrix([ [0.0],
                    [0.0],
                    [0.0],
                    [i/delta]])
    fx,fu,fxx,fuu,fux = model.computeAllModelDeriv(dt,X,U)
    lastX = nextX
    nextX = model.computeNextState(dt,X,U)

    if(i< N-3 ):
        analyticList0o2.append(fxx[3][0,3].item())
        analyticList1o2.append(fxx[3][1,3].item())
        analyticList2o2.append(fxx[3][2,3].item())
        analyticList3o2.append(fxx[3][3,3].item())

        diffList0o2.append((diffList0[i+1]-diffList0[i])*(delta))
        diffList1o2.append((diffList1[i+1]-diffList1[i])*(delta))
        diffList2o2.append((diffList2[i+1]-diffList2[i])*(delta))
        diffList3o2.append((diffList3[i+1]-diffList3[i])*(delta))

        err0o2.append(fxx[3][0,3].item() - (diffList0[i+1]-diffList0[i])*(delta))
        err1o2.append(fxx[3][1,3].item() - (diffList1[i+1]-diffList1[i])*(delta))
        err2o2.append(fxx[3][2,3].item() - (diffList2[i+1]-diffList2[i])*(delta))
        err3o2.append(fxx[3][3,3].item() - (diffList3[i+1]-diffList3[i])*(delta))



print analyticList0
print diffList0
print "\n"
print analyticList1
print diffList1
print "\n"
print analyticList2
print diffList2
print "\n"
print analyticList3
print diffList3
print "\n"


fig2 = pl.figure ()
ax2 = fig2.add_subplot ('221')
ax2.set_title('tau ')
ax2.plot ( range(N-3),analyticList0o2,
          range(N-3),diffList0o2,'r')
bx2 = fig2.add_subplot ('222')
bx2.set_title('tauDot')
bx2.plot ( range(N-3),analyticList1o2,
          range(N-3),diffList1o2,'r')
cx2 = fig2.add_subplot ('223')
cx2.set_title('q')
cx2.plot ( range(N-3),analyticList2o2,
          range(N-3),diffList2o2,'r')
dx2 = fig2.add_subplot ('224')
dx2.set_title('qDot')
dx2.plot ( range(N-3),analyticList3o2,
          range(N-3),diffList3o2,'r')
ax2.grid()
bx2.grid()
cx2.grid()
dx2.grid()

fig3 = pl.figure()
ax3 = fig3.add_subplot ('221')
ax3.set_title('tau ')
ax3.plot ( range(N-3),err0o2)
bx3 = fig3.add_subplot ('222')
bx3.set_title('tauDot')
bx3.plot ( range(N-3),err1o2)
cx3 = fig3.add_subplot ('223')
cx3.set_title('q')
cx3.plot ( range(N-3),err2o2)
dx3 = fig3.add_subplot ('224')
dx3.set_title('qDot')
dx3.plot ( range(N-3),err3o2)
ax3.grid()
bx3.grid()
cx3.grid()
dx3.grid()


for i in range(N):
    X = np.matrix([ [0.0],
                    [0.0],
                    [0.0],
                    [0.0]])
    U = i/delta
    fx,fu,fxx,fuu,fux = model.computeAllModelDeriv(dt,X,U)
    lastX = nextX
    nextX = model.computeNextState(dt,X,U)

    if((i>0) & (i< N-1) ):
        analyticList0U.append(fu[0].item())
        analyticList1U.append(fu[1].item())
        analyticList2U.append(fu[2].item())
        analyticList3U.append(fu[3].item())
        diffList0U.append((nextX[0].item()-lastX[0].item())*(delta))
        diffList1U.append((nextX[1].item()-lastX[1].item())*(delta))
        diffList2U.append((nextX[2].item()-lastX[2].item())*(delta))
        diffList3U.append((nextX[3].item()-lastX[3].item())*(delta))

        err0U.append(fu[0].item() - (nextX[0].item()-lastX[0].item())*(delta))
        err1U.append(fu[1].item() - (nextX[1].item()-lastX[1].item())*(delta))
        err2U.append(fu[2].item() - (nextX[2].item()-lastX[2].item())*(delta))
        err3U.append(fu[3].item() - (nextX[3].item()-lastX[3].item())*(delta))

fig4 = pl.figure()
ax4 = fig4.add_subplot('221')
ax4.set_title("tau")
ax4.plot(range(N-2), analyticList0U,
            range(N-2),diffList0U)
bx4 = fig4.add_subplot('222')
bx4.set_title("tauDot")
bx4.plot(range(N-2), analyticList1U,
            range(N-2),diffList1U)
cx4 = fig4.add_subplot('223')
cx4.set_title("q")
cx4.plot(range(N-2), analyticList2U,
            range(N-2),diffList2U)
dx4 = fig4.add_subplot('224')
dx4.set_title("qDot")
dx4.plot(range(N-2), analyticList3U,
            range(N-2),diffList3U)
ax4.grid()
bx4.grid()
cx4.grid()
dx4.grid()

fig5 = pl.figure()
ax5 = fig5.add_subplot('221')
ax5.set_title("tau")
ax5.plot(range(N-2), err0U)
bx5 = fig5.add_subplot('222')
bx5.set_title("tauDot")
bx5.plot(range(N-2), err1U)
cx5 = fig5.add_subplot('223')
cx5.set_title("q")
cx5.plot(range(N-2), err2U)
dx5 = fig5.add_subplot('224')
dx5.set_title("qDot")
dx5.plot(range(N-2), err3U)
ax5.grid()
bx5.grid()
cx5.grid()
dx5.grid()




pl.show()