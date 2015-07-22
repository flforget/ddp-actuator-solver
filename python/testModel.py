__author__ = 'fforget'

import numpy as np
from matplotlib import pyplot as pl
from dynamicModel import *
from scipy.integrate import odeint

model = simpleRomeoActuatorDynamicModel()

dt = 1e-4
X = np.matrix([[1.0,0.0,0.0,0.0]]).T
U = 0.0


# lists init
XList = list()
XrealList = list()
tauList = list()
tauDotList = list()
qList = list()
qDotList = list()
taurealList = list()
tauDotrealList = list()
qrealList = list()
qDotrealList = list()
errList = list()
fxList = list()
XList.append(X)
XrealList.append(X)
tauList.append(X[0].item())
tauDotList.append(X[1].item())
qList.append(X[2].item())
qDotList.append(X[3].item())
taurealList.append(X[0].item())
tauDotrealList.append(X[1].item())
qrealList.append(X[2].item())
qDotrealList.append(X[3].item())
errList.append(0.0)


N = 1000
for i in range(N):
    X = model.computeNextState(dt,X,U)
    XList.append(X)
    tauList.append(X[0].item())
    tauDotList.append(X[1].item())
    qList.append(X[2].item())
    qDotList.append(X[3].item())


    # solving ODE to compare discrete model with continuous one
    time = np.linspace((i)*dt, (i+1)*dt)
    # example for odeint -> Xreal = odeint(romeoActuator,[X[0].item(),X[1].item(),X[2].item(),X[3].item()],time)
    Xrealfull = odeint(model.continuousModel,[X[0].item(),X[1].item(),X[2].item(),X[3].item()],time,args=(U,))
    Xreal = Xrealfull[Xrealfull.shape[0]-1,:]
    XrealList.append(Xreal)
    taurealList.append(Xreal[0].item())
    tauDotrealList.append(Xreal[1].item())
    qrealList.append(Xreal[2].item())
    qDotrealList.append(Xreal[3].item())

    errList.append(X[0].item()-Xreal[0].item())

fig = pl.figure ()
ax = fig.add_subplot ('311')
ax.set_title('tau (b) - real (dot) ')
ax.plot ( range(N+1),tauList,
          range(N+1),taurealList,':')
bx = fig.add_subplot ('312')
bx.set_title('error continuous-discrete')
bx.plot ( range(N+1),errList)
cx = fig.add_subplot ('313')
cx.set_title('angle q (b) - angular velocity (r) - real (dot)')
cx.plot ( range(N+1),qList,'b', range(N+1),qDotList,'r',
          range(N+1),qrealList,'b:', range(N+1),qDotrealList,'r:')
ax.grid()
bx.grid()
cx.grid()




pl.show()