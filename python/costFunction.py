import numpy as np
from tools import nRoot

class CostFunctionSimpleRomeoActuator:
    def __init__(self):
        self.Q = np.matrix([[100.0,0.0,0.0,0.0],
                            [0.0,0.0,0.0,0.0],
                            [0.0,0.0,0.0,0.0],
                            [0.0,0.0,0.0,0.0]])
        self.R = np.matrix([[0.1]])

        self.lxx = self.Q
        self.luu = self.R
        self.lux = np.zeros((1,4))
        self.lxu = np.zeros((4,1))

    def computeCostValue(self,T,XList,Xdes,UList):
        cost = 0.0
        for i in range(T):
            X = np.matrix(XList[i])
            U = np.matrix(UList[i])
            cost += 0.5*(X-Xdes).T*self.Q*(X-Xdes) \
                   + 0.5 * U*self.R*U
        cost += 1*(XList[T]-Xdes).T*self.Q*(XList[T]-Xdes)
        return cost

    def computeAllCostDeriv(self,X,Xdes,U):
        lx = self.Q*(X-Xdes)
        lu = self.R*U

        return lx,self.lxx,lu,self.luu,self.lxu,self.lux

    def computeFinalCostDeriv(self,X,Xdes):
        lx = 1.0*self.Q*(X-Xdes)
        return lx,self.lxx


class CostFunctionSimpleRomeoActuatorWithPowerLimit:
    def __init__(self):
        self.Q = np.matrix([[100.0,0.0,0.0,0.0],
                            [0.0,0.0,0.0,0.0],
                            [0.0,0.0,0.0,0.0],
                            [0.0,0.0,0.0,0.0]])
        self.R = np.matrix([[0.1]])

    def computeCostValue(self,T,XList,Xdes,UList):
        cost = 0.0
        for i in range(T):
            X = np.matrix(XList[i])
            U = np.matrix(UList[i])
            cost += 0.5*(X-Xdes).T*self.Q*(X-Xdes) \
                   + 0.5 * U*self.R*U
        cost += 1*(XList[T]-Xdes).T*self.Q*(XList[T]-Xdes)
        return cost

    def computeAllCostDeriv(self,X,Xdes,U):
        lx = self.Q*(X-Xdes)
        lxx = self.Q
        lu = self.R*U
        luu = self.R
        lux = np.zeros((1,4))
        lxu = np.zeros((4,1))

        return lx,lxx,lu,luu,lux,lxu

    def computeFinalCostDeriv(self,X,Xdes):
        lx = 1.0*self.Q*(X-Xdes)
        lxx = 1.0*self.Q
        return lx,lxx


class CostFunctionMCC3States:
    def __init__(self):
        # X = (theta,omega,tau).T
        self.Q = np.matrix([[0.001,0.0,0.0],
                            [0.0,0.001,0.0],
                            [0.0,0.0,100.0]])
        self.R = np.matrix([[0.1]])

    def computeCostValue(self,T,XList,Xdes,UList):
        cost = 0.0
        for i in range(T):
            X = np.matrix(XList[i])
            U = np.matrix(UList[i])
            cost += 0.5*(X-Xdes).T*self.Q*(X-Xdes) + 0.5*U.T*self.R*U
        cost += 0.5*(XList[T]-Xdes).T*self.Q*(XList[T]-Xdes)
        return cost

    def computeAllCostDeriv(self,X,Xdes,U):
        lx = self.Q*(X-Xdes)
        lxx = self.Q
        lu = self.R*U
        luu = self.R
        lux = np.zeros((1,3))
        lxu = np.zeros((3,1))

        return lx,lxx,lu,luu,lux,lxu

    def computeFinalCostDeriv(self,X,Xdes):
        lx = 1*self.Q*(X-Xdes)
        lxx = 1*self.Q
        return lx,lxx


class CostFunctionMCC2states:
    def __init__(self):
        # X = (theta,omega,tau).T
        self.Q = np.matrix([[0.0,0.0],
                            [0.0,100.0]])
        self.R = np.matrix([[0.1]])

    def computeCostValue(self,T,XList,Xdes,UList):
        cost = 0.0
        for i in range(T):
            X = np.matrix(XList[i])
            U = np.matrix(UList[i])
            cost += 0.5*(X-Xdes).T*self.Q*(X-Xdes) \
                    + 0.5*U.T*self.R*U
        cost += 0.5*(XList[T]-Xdes)*self.Q*(XList[T]-Xdes)
        return cost

    def computeAllCostDeriv(self,X,Xdes,U):
        lx = self.Q*(X-Xdes)
        lxx = self.Q
        lu = self.R*U
        luu = self.R
        lux = np.zeros((1,2))
        lxu = np.zeros((2,1))

        return lx,lxx,lu,luu,lux,lxu

    def computeFinalCostDeriv(self,X,Xdes):
        lx = 1.0*self.Q*(X-Xdes)
        lxx = 1.0*self.Q
        return lx,lxx


class CostFunctionRandom:
    def __init__(self):
        self.Q = np.matrix([[100.0,0.0,0.0,0.0],
                            [0.0,0.0,0.0,0.0],
                            [0.0,0.0,0.0,0.0],
                            [0.0,0.0,0.0,0.0]])
        self.R = np.matrix([[0.1,0.0],
                            [0.0 ,0.1]])

    def computeCostValue(self,T,XList,Xdes,UList):
        cost = 0.0
        for i in range(T):
            X = np.matrix(XList[i])
            U = np.matrix(UList[i])
            cost += 0.5*(X-Xdes).T*self.Q*(X-Xdes) \
                   + 0.5*U.T*self.R*U
        cost += 1.0*(XList[T]-Xdes).T*self.Q*(XList[T]-Xdes)
        return cost

    def computeAllCostDeriv(self,X,Xdes,U):
        lx = self.Q*(X-Xdes)
        lxx = self.Q
        lu = self.R*U
        luu = self.R
        lux = np.zeros((2,4))
        lxu = np.zeros((4,2))

        return lx,lxx,lu,luu,lux,lxu

    def computeFinalCostDeriv(self,X,Xdes):
        lx = 1.0*self.Q*(X-Xdes)
        lxx = 1.0*self.Q
        return lx,lxx