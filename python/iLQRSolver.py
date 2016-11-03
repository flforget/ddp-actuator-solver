import numpy as np
import numpy.linalg
import matplotlib.pyplot as pl
from costFunction import *
from dynamicModel import *
import time

class ILQRSolver:
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

        XList,UList = self.initTrajectory()
        for iter in range(self.iterMax):
            kList,KList = self.backwardLoop(XList,UList)
            newXList,newUList = self.forwardLoop(kList,KList,XList,UList)
            XList = newXList
            UList = newUList

            if(self.changeAmount < self.stopCrit):
                break

        return XList,UList,iter


    def initTrajectory(self):
        XList = []
        append = XList.append
        computeNextState = self.model.computeNextState
        zerosCommand = np.zeros((self.model.commandNumber,1))
        dt = self.dt
        UList = [zerosCommand for i in range(self.T)]
        append(self.Xinit)
        for i in range(self.T):
            append(computeNextState(dt,XList[i],UList[i]))
        return XList,UList


    def computeAllDeriv(self,X,Xdes,U):
        ''' useless '''
        fx,fxx,fu,fuu,fxu,fux = self.model.computeAllModelDeriv(self.dt,X,U)
        lx,lxx,lu,luu,lxu,lux = self.costfunction.computeAllCostDeriv(X,Xdes,U)
        return fx,fxx,fu,fuu,fxu,fux,lx,lxx,lu,luu,lxu,lux


    def backwardLoop(self,XList,UList):
        computeAllModelDeriv = self.model.computeAllModelDeriv
        computeAllCostDeriv = self.costfunction.computeAllCostDeriv
        kList = []
        KList = []
        kappend = kList.append
        Kappend = KList.append
        nextVx,nextVxx = \
            self.costfunction.computeFinalCostDeriv(XList[self.T],self.Xdes)

        Xdes = self.Xdes
        stateNb = self.model.stateNumber
        dt = self.dt

        mu = 0.0
        completeBackwardFlag = 0
        while(completeBackwardFlag==0):
            completeBackwardFlag = 1
            muEye = mu*np.eye( nextVxx.shape[0] , dtype=float)
            for i in range(self.T-1,-1,-1):  # backward pass
                X = XList[i]
                U = UList[i]

                fx,fxx,fu,fuu,fxu,fux = computeAllModelDeriv(dt,X,U)
                lx,lxx,lu,luu,lxu,lux = computeAllCostDeriv(X,Xdes,U)

                Qx = lx + fx.T*nextVx
                Qu = lu + fu.T*nextVx
                Qxx = lxx + fx.T*nextVxx*fx
                # Quu = luu + fu.T*nextVxx*fu
                # Qux = lux + fu.T*nextVxx*fx
                # Qxu = lxu + fx.T*nextVxx*fu

                Quut = luu + fu.T*(nextVxx+muEye)*fu
                Quxt = lux + fu.T*(nextVxx+muEye)*fx
                for j in range(stateNb): # second order derivative of dynamic
                    Qxx += nextVx[j].item()*fxx[j]
                    # Qux += nextVx[j].item()*fux[j]
                    Quxt += nextVx[j].item()*fux[j]
                    # Quu += nextVx[j].item()*fuu[j]
                    Quut += nextVx[j].item()*fuu[j]


                if(np.all(np.linalg.eigvals(Quut) <= 0)): # check out if Quut is definite positive
                    if(mu==0):
                        mu += 1e-4
                    else:
                        mu = mu*10
                    completeBackwardFlag = 0
                    break

                QuuInv = np.linalg.inv(Quut)

                k = - QuuInv*Qu
                K = - QuuInv*Quxt

                # regularization (Y. Tassa thesis)
                # nextVx      = Qx + K.T*Quut*k + K.T*Qu + Quxt.T*k
                # nextVxx     = Qxx + K.T*Quut*K + K.T*Quxt + Quxt.T*K
                nextVx = Qx - K.T*Quut*k
                nextVxx = Qxx - K.T*Quut*K

                kappend(k)
                Kappend(K)
                # kList[i] = k
                # KList[i] = K
        kList.reverse()
        KList.reverse()
        return kList,KList

    def forwardLoop(self,kList,KList,XList,UList):
        newXList = []
        newUList = []
        xappend = newXList.append
        uappend = newUList.append
        computeNextState = self.model.computeNextState
        changeAmount = 0.0
        dt = self.dt
        commandNb = self.model.commandNumber

        xappend(self.Xinit)
        # Line search to be implemented
        alphaList = [1.0,0.8,0.6,0.4,0.2]
        alpha = alphaList[0]
        for i in range(self.T):
            uappend( UList[i] + alpha*kList[i] \
                        + KList[i]*(newXList[i] - XList[i]))
            xappend(computeNextState(dt,newXList[i],newUList[i]))
            for j in range(commandNb):
                U = UList[i]
                newU = newUList[i]
                changeAmount += np.abs(U[j] - newU[j])
        self.changeAmount = changeAmount
        return newXList,newUList