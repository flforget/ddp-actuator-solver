import numpy as np
import numpy.linalg
import matplotlib.pyplot as pl
from costFunction import *
from dynamicModel import *
import time

class ILQRSolverImproved:
    def __init__(self,model,costFunction):
        self.model = model
        self.costfunction = costFunction
        ''' init values changed when solveTrajectory is called '''
        self.xinit = np.zeros((model.stateNumber,1))
        self.Xdes = np.zeros((model.stateNumber,1))
        self.T = 10
        self.dt = 1e-4
        self.iterMax = 20
        self.stopCrit = 1e-3
        ''' '''
        self.changeAmount = 0.0
        self.completeBackwardFlag = 0

        self.X = np.zeros((self.model.stateNumber,1))
        self.nextX = np.zeros((self.model.stateNumber,1))
        self.U = np.zeros((self.model.commandNumber,1))
        self.nextXList = []
        self.nextUList = []
        self.Qx = np.zeros((self.model.stateNumber,1))
        self.Qu = np.zeros((self.model.commandNumber,1))
        self.Qxx = np.zeros((self.model.stateNumber,self.model.stateNumber))
        self.Quu = np.zeros((self.model.commandNumber,self.model.commandNumber))

        self.alphaList = [1.0,0.8,0.6,0.4,0.2]
        self.alpha = 1.0
        self.mu = 0.0
        self.muEye = self.mu*np.eye( self.model.stateNumber , dtype=float)

        self.zerosCommand = np.zeros((self.model.commandNumber,1))

        self.kList = []
        self.KList = []
        self.XList = []
        self.UList = []
        self.k = np.zeros((self.model.commandNumber,1))
        self.K = np.zeros((self.model.commandNumber,self.model.stateNumber))
        self.Qx = np.zeros((self.model.stateNumber,1))
        self.Qu = np.zeros((self.model.commandNumber,1))
        self.Qxx = np.zeros((self.model.stateNumber,self.model.stateNumber))
        self.Quut = np.zeros((self.model.commandNumber,self.model.commandNumber))
        self.Quxt = np.zeros((self.model.stateNumber,self.model.commandNumber))
        self.Quuinv = np.zeros((self.model.commandNumber,self.model.commandNumber))
        self.nextVx = np.zeros((self.model.stateNumber,1))
        self.nextVxx = np.zeros((self.model.stateNumber,self.model.stateNumber))


    def solveTrajectory(self,Xinit,Xdes,T,dt,iterMax=20,stopCrit=1e-3):
        """
        _ initialisation
        _ backward Pass
        _ optimization of regularization coef
        _ forward Pass with optimization of line-search coef
        _ stop when convergence or iterMax reached
        """
        self.Xinit = Xinit
        self.Xdes = Xdes
        self.T = T
        self.dt = dt
        self.iterMax = iterMax
        self.stopCrit = stopCrit


        self.initTrajectory()
        for iter in range(self.iterMax):
            self.backwardLoop()
            self.forwardLoop()
            self.XList = self.nextXList
            self.UList = self.nextUList
            if(self.changeAmount < self.stopCrit):
                break
        return self.XList,self.UList


    def initTrajectory(self):
        self.XList = [self.Xinit]
        self.UList = [self.zerosCommand for i in range(self.T)]
        for i in range(self.T):
            self.model.computeNextState(self.dt,self.XList[i],self.UList[i])
            self.XList.append(self.model.nextX)
        return 0


    def computeAllDeriv(self,X,Xdes,U):
        '''useless'''
        self.model.computeAllModelDeriv(self.dt,X,U)
        self.costfunction.computeAllCostDeriv(X,Xdes,U)


    def backwardLoop(self):
        self.kList = []
        self.KList = []
        self.Xdes
        self.costfunction.computeFinalCostDeriv(self.XList[self.T],self.Xdes)
        self.nextVx = self.costfunction.lx
        self.nextVxx = self.costfunction.lxx
        self.mu = 0.0
        self.completeBackwardFlag = 0
        while(self.completeBackwardFlag==0):
            self.completeBackwardFlag = 1
            self.muEye = self.mu*np.eye( self.nextVxx.shape[0] , dtype=float)
            for i in range(self.T-1,-1,-1):  # backward pass
                self.X = self.XList[i]
                self.U = self.UList[i]

                self.model.computeAllModelDeriv(self.dt,self.X,self.U)
                self.costfunction.computeAllCostDeriv(self.X,self.Xdes,self.U)

                self.Qx = self.costfunction.lx + np.dot(self.model.fx.T,self.nextVx)
                self.Qu = self.costfunction.lu + np.dot(self.model.fu.T,self.nextVx)
                self.Qxx = self.costfunction.lxx \
                           + np.dot(np.dot(self.model.fx.T,self.nextVxx),self.model.fx)
                self.Quut = self.costfunction.luu \
                            + np.dot(np.dot(self.model.fu.T,(self.nextVxx+self.muEye)),self.model.fu)
                self.Quxt = self.costfunction.lux \
                            + np.dot(np.dot(self.model.fu.T,(self.nextVxx+self.muEye)),self.model.fx)

                for j in range(self.model.stateNumber): # second order derivative of dynamic
                    self.Qxx += np.dot(self.nextVx[j].item(),self.model.fxx[j])
                    self.Quxt += np.dot(self.nextVx[j].item(),self.model.fux[j])
                    self.Quut += np.dot(self.nextVx[j].item(),self.model.fuu[j])

                if(np.all(np.linalg.eigvals(self.Quut) <= 0)): # check out if Quut is definite positive
                    if(self.mu==0):
                        self.mu += 1e-4
                    else:
                        self.mu = self.mu*10
                    self.completeBackwardFlag = 0
                    break

                self.QuuInv = np.linalg.inv(self.Quut)

                self.k = - np.dot(self.QuuInv,self.Qu)
                self.K = - np.dot(self.QuuInv,self.Quxt)

                # regularization (Y. Tassa thesis)
                self.nextVx      = self.Qx + np.dot(np.dot(self.K.T,self.Quut),self.k) \
                                   + np.dot(self.K.T,self.Qu) + np.dot(self.Quxt.T,self.k)
                self.nextVxx     = self.Qxx + np.dot(np.dot(self.K.T,self.Quut),self.K) \
                                   + np.dot(self.K.T,self.Quxt) + np.dot(self.Quxt.T,self.K)
                # nextVx = Qx - K.T*Quu*k
                # nextVxx = Qxx - K.T*Quu*K

                self.kList.append(self.k)
                self.KList.append(self.K)
        self.kList.reverse()
        self.KList.reverse()
        return 0

    def forwardLoop(self):
        self.nextXList = [self.Xinit]
        self.nextUList = []

        self.changeAmount = 0.0
        self.nextXList[0] = self.Xinit
        # Line search to be implemented
        self.alpha = self.alphaList[0]
        for i in range(self.T):
            self.nextUList.append(self.UList[i] + self.alpha*self.kList[i] \
                        + np.dot(self.KList[i],(self.nextXList[i] - self.XList[i])))
            self.model.computeNextState(self.dt,self.nextXList[i],self.nextUList[i])
            self.nextXList.append(self.model.nextX)
            for j in range(self.model.commandNumber):
                self.changeAmount += np.abs(self.UList[j] - self.nextUList[j])
        return 0
