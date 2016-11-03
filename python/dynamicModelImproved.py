import numpy as np
from random import uniform
import time

class SimpleRomeoActuatorDynamicModelImproved:
    """ discrete dynamic model definition class
            _ return derivatives
            _ return nextState from currentState
            _ input of iLQRSolver
    """

    def __init__(self): # constructor
        self.X = np.zeros((4,1))
        self.U = 0.0
        self.nextX = np.zeros((4,1))

        self.tau = 0.0
        self.tauDot = 0.0
        self.q = 0.0
        self.qDot = 0.0

        self.k = 1000 # 200*1e9*19.64*1e-6*0.08/0.1         # raideur k = E*S/l   (E = 200000 N/mm2)
        self.R = 200.0                                      # rapport de reduction
        self.Jm = 138*1e-7                                  # inertie moteur
        self.Jl = 0.1                                       # inertie charge
        self.fvm = 0.01#0.00001                             # frottements visqueux moteur
        self.Cf0 = 0.1
        self.a = 10.0

        self.A10 = (self.k/self.Jl) + (self.k/(self.Jm*self.R*self.R))
        self.A11 = self.fvm/self.Jm
        self.A13 = self.fvm*self.k/self.Jm
        self.A13arctan = (2*self.Jm*self.R/(np.pi*self.Jl))*self.Cf0
        self.A30 = 1.0/self.Jl
        self.A33arctan = (2/(np.pi*self.Jl))*self.Cf0
        self.B2 = self.k/(self.R*self.Jm)


        self.stateNumber = 4
        self.commandNumber = 1

        self.fx10 = -(self.k/self.Jl) - (self.k/(self.Jm*self.R*self.R))
        self.fx11 = self.fvm/self.Jm
        self.fx12 = self.fvm*self.k/self.Jm
        self.fu = np.matrix([[0.0],
                        [0.0],
                        [0.0],
                        [0.0]])
        self.fxx = list()
        self.fxx.append( np.zeros((4,4)))
        self.fxx.append( np.zeros((4,4)))
        self.fxx.append( np.zeros((4,4)))
        self.fxx.append( np.zeros((4,4)))
        self.fuu = np.matrix([  [0.0],
                                [0.0],
                                [0.0],
                                [0.0]  ])
        self.fux = np.matrix([  [0.0,0.0,0.0,0.0],
                                [0.0,0.0,0.0,0.0],
                                [0.0,0.0,0.0,0.0],
                                [0.0,0.0,0.0,0.0]])
        self.fxu = np.matrix([  [0.0,0.0,0.0,0.0],
                                [0.0,0.0,0.0,0.0],
                                [0.0,0.0,0.0,0.0],
                                [0.0,0.0,0.0,0.0]])


    def computeNextState(self,dt,X,U): # compute state for next time step
        tau = X[0,0] + X[1,0]*dt
        tauDot = X[1,0] + dt*(self.B2*U \
                                - self.A10*X[0,0] \
                                - self.A11*X[1,0] \
                                - self.A13*X[3,0] \
                                + self.A13arctan*np.arctan(self.a*X[3,0]))
        q = X[2,0] + X[3,0]*dt
        qDot = X[3,0] + dt*(self.A30*X[0,0] - self.A33arctan*np.arctan(self.a*X[3,0]))

        self.nextX = np.matrix([ [tau],
                            [tauDot],
                            [q],
                            [qDot]  ])
        return 0


    def computeAllModelDeriv(self,dt,X,U):
        self.fx = np.matrix([[1.0,dt,0.0,0.0],
                        [dt*self.fx10,
                         1 - dt*self.fx11,
                         0.0,
                        -dt*self.fx12
                        + ((2*dt*self.Jm*self.R)/(np.pi*self.Jl))*self.Cf0*(self.a/(1+(self.a*self.a*X[3]*X[3])))],
                        [0.0,0.0,1.0,dt],
                        [dt/self.Jl,0.0,0.0,1-((2*dt*self.Cf0)/(np.pi*self.Jl))*(self.a/(1+(self.a*self.a*X[3]*X[3])))]])

        self.fu[1,0] = dt*self.k/(self.R*self.Jm)

        self.fxx[3][1,3] = -((2*dt*self.Jm*self.R)/(np.pi*self.Jl))*self.Cf0*((2*self.a*self.a*self.a*X[3])/((1+(self.a*self.a*X[3]*X[3]))*(1+(self.a*self.a*X[3]*X[3]))))
        self.fxx[3][3,3] = +((2*dt*self.Cf0)/(np.pi*self.Jl))*((2*self.a*self.a*self.a*X[3])/((1+(self.a*self.a*X[3]*X[3]))*(1+(self.a*self.a*X[3]*X[3]))))
        return 0
