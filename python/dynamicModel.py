import numpy as np
from random import uniform

class SimpleRomeoActuatorDynamicModel:
    """ discrete dynamic model definition class
            _ return derivatives
            _ return nextState from currentState
            _ input of iLQRSolver
    """
    def __init__(self): # constructor
        self.k = 1000 # 200*1e9*19.64*1e-6*0.08/0.1         # raideur k = E*S/l   (E = 200000 N/mm2)
        self.R = 200.0                                      # rapport de reduction
        self.Jm = 138*1e-7                                  # inertie moteur
        self.Jl = 0.1                                       # inertie charge
        self.fvm = 0.01#0.00001                             # frottements visqueux moteur
        self.Cf0 = 0.1
        self.a = 10.0

        self.stateNumber = 4
        self.commandNumber = 1

        self.fu = np.matrix([   [0.0],
                                [0.0],
                                [0.0],
                                [0.0]])
        self.fuu = np.matrix([  [0.0],
                                [0.0],
                                [0.0],
                                [0.0]  ])
        self.fxx = list(range(4))
        self.fxx[0] = np.zeros((4,4))
        self.fxx[1] = np.zeros((4,4))
        self.fxx[2] = np.zeros((4,4))
        self.fxx[3] = np.zeros((4,4))
        self.fux = np.zeros((4,4))
        self.fxu = np.zeros((4,4))

    def computeNextState(self,dt,X,U): # compute state for next time step
        k = self.k
        R = self.R
        Jm = self.Jm
        Jl = self.Jl
        fvm = self.fvm
        Cf0 = self.Cf0
        a = self.a

        tau = X[0,0]
        tauDot = X[1,0]
        q = X[2,0]
        qDot = X[3,0]

        nextTau = tau + tauDot*dt
        nextTauDot = tauDot + dt*((k/(R*Jm))*U - tau*((k/Jl) + (k/(Jm*R*R))) - tauDot*(fvm/Jm) - qDot*(fvm*k/Jm) + (2*Jm*R/(np.pi*Jl))*Cf0*np.arctan(a*qDot))
        nextQ = q + qDot*dt
        nextQDot = qDot + dt*(tau/Jl - (2/(np.pi*Jl))*Cf0*np.arctan(a*qDot))

        nextX = np.matrix([ [nextTau],
                            [nextTauDot],
                            [nextQ],
                            [nextQDot]  ])
        return nextX

    def computeNextStateComprehension(self,dt,XU): # compute state for next time step
        k = self.k
        R = self.R
        Jm = self.Jm
        Jl = self.Jl
        fvm = self.fvm
        Cf0 = self.Cf0
        a = self.a

        tau = XU[0,0]
        tauDot = XU[1,0]
        q = XU[2,0]
        qDot = XU[3,0]

        nextTau = tau + tauDot*dt
        nextTauDot = tauDot + dt*((k/(R*Jm))*XU[4,0] - tau*((k/Jl) + (k/(Jm*R*R))) - tauDot*(fvm/Jm) - qDot*(fvm*k/Jm) + (2*Jm*R/(np.pi*Jl))*Cf0*np.arctan(a*qDot))
        nextQ = q + qDot*dt
        nextQDot = qDot + dt*(tau/Jl - (2/(np.pi*Jl))*Cf0*np.arctan(a*qDot))

        nextX = np.matrix([ [nextTau],
                            [nextTauDot],
                            [nextQ],
                            [nextQDot]  ])
        return nextX


    def computeAllModelDeriv(self,dt,X,U):
        qDot = X[3,0]
        k = self.k
        R = self.R
        Jm = self.Jm
        Jl = self.Jl
        fvm = self.fvm
        Cf0 = self.Cf0
        a = self.a

        fx = np.matrix([[1.0,dt,0.0,0.0],
                        [dt*(-(k/Jl) - (k/(Jm*R*R))), 1 - dt*(fvm/Jm),0.0,
                            -dt*(fvm*k/Jm) + ((2*dt*Jm*R)/(np.pi*Jl))*Cf0*(a/(1+(a*a*qDot*qDot)))],
                        [0.0,0.0,1.0,dt],
                        [dt/Jl,0.0,0.0,1-((2*dt*Cf0)/(np.pi*Jl))*(a/(1+(a*a*qDot*qDot)))]])
        self.fu[1,0] = dt*k/(R*Jm)


        self.fxx[3][1,3] = -((2*dt*Jm*R)/(np.pi*Jl))*Cf0*((2*a*a*a*qDot)/((1+(a*a*qDot*qDot))*(1+(a*a*qDot*qDot))))
        self.fxx[3][3,3] = +((2*dt*Cf0)/(np.pi*Jl))*((2*a*a*a*qDot)/((1+(a*a*qDot*qDot))*(1+(a*a*qDot*qDot))))

        return fx,self.fxx,self.fu,self.fuu,self.fxu,self.fux


class MCCmodel:
    def __init__(self):
        self.f = 0.001
        self.J = 0.001
        self.L = 0.001
        self.r = 1.0
        self.k = 1.0
        # X = (theta,omega,tau).T
        self.A = np.matrix([[0.0,1.0,0.0],
                            [0.0,-self.f/self.J,1.0/self.J],
                            [0.0,-1.0/self.L,-self.r/self.L]])
        self.B = np.matrix([[0.0],
                            [0.0],
                            [self.k/self.L]])


        self.stateNumber = 3
        self.commandNumber = 1

    def computeNextState(self,dt,X,U):
        Ad = dt*(self.A)+np.eye(3)
        Bd = dt*self.B
        return Ad*X+Bd*U

    def computeAllModelDeriv(self,dt,X,U):
        fx = dt*(self.A)+np.eye(3)
        fxx= list()
        fxx.append(np.zeros((3,3)))
        fxx.append(np.zeros((3,3)))
        fxx.append(np.zeros((3,3)))
        fu = self.B*dt
        fuu = np.zeros((3,1))
        fux = np.zeros((3,3))
        fxu = np.zeros((3,3))
        return fx,fxx,fu,fuu,fxu,fux


class simplifiedMCCmodel:
    def __init__(self):
        self.f = 0.001
        self.J = 0.001
        self.L = 0.001
        self.r = 1.0
        self.k = 1.0
        # X = (theta,omega,tau).T
        self.A = np.matrix([[-self.f/self.J         ,   1.0/self.J],
                            [-self.k*self.k/self.L  ,   -self.r/self.L]])
        self.B = np.matrix([[0.0],
                            [self.k/self.L]])


        self.stateNumber = 2
        self.commandNumber = 1

    def computeNextState(self,dt,X,U):
        Ad = dt*(self.A)+np.eye(2)
        Bd = dt*self.B
        return Ad*X+Bd*U

    def computeAllModelDeriv(self,dt,X,U):
        fx = dt*(self.A)+np.eye(2)
        fxx= list()
        fxx.append(np.zeros((2,2)))
        fxx.append(np.zeros((2,2)))
        fu = self.B*dt
        fuu = np.zeros((2,1))
        fux = np.zeros((2,2))
        fxu = np.zeros((2,2))
        return fx,fxx,fu,fuu,fxu,fux


class randomModel:
    def __init__(self):
        self.A = np.matrix([[uniform(-1000,1000),uniform(-1000,1000),uniform(-1000,1000),uniform(-1000,1000)],
                            [uniform(-1000,1000),uniform(-1000,1000),uniform(-1000,1000),uniform(-1000,1000)],
                            [uniform(-1000,1000),uniform(-1000,1000),uniform(-1000,1000),uniform(-1000,1000)],
                            [uniform(-1000,1000),uniform(-1000,1000),uniform(-1000,1000),uniform(-1000,1000)]])
        self.B = np.matrix([[uniform(-1000,1000),uniform(-1000,1000)],
                            [uniform(-1000,1000),uniform(-1000,1000)],
                            [uniform(-1000,1000),uniform(-1000,1000)],
                            [uniform(-1000,1000),uniform(-1000,1000)]])


        self.stateNumber = 4
        self.commandNumber = 2

    def computeNextState(self,dt,X,U):
        Ad = dt*(self.A)+np.eye(4)
        Bd = dt*self.B
        return Ad*X+Bd*U

    def computeAllModelDeriv(self,dt,X,U):
        fx = dt*(self.A)+np.eye(4)
        fxx= list()
        fxx.append(np.zeros((4,4)))
        fxx.append(np.zeros((4,4)))
        fxx.append(np.zeros((4,4)))
        fxx.append(np.zeros((4,4)))
        fu = self.B*dt
        fuu = list()
        fuu.append(np.zeros((2,2)))
        fuu.append(np.zeros((2,2)))
        fuu.append(np.zeros((2,2)))
        fuu.append(np.zeros((2,2)))
        fux = list()
        fux.append(np.zeros((2,4)))
        fux.append(np.zeros((2,4)))
        fux.append(np.zeros((2,4)))
        fux.append(np.zeros((2,4)))
        fxu = list()
        fxu.append(np.zeros((4,4)))
        fxu.append(np.zeros((4,4)))
        return fx,fxx,fu,fuu,fxu,fux