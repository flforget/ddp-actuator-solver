import numpy as np

class CostFunctionSimpleRomeoActuator:
    def __init__(self):
        self.Q = np.matrix([[100.0,0.0,0.0,0.0],
                        [0.0,0.0,0.0,0.0],
                        [0.0,0.0,0.0,0.0],
                        [0.0,0.0,0.0,0.0]])
        self.R = np.matrix([[0.1]])

        self.lx = np.zeros((4,1))
        self.lu = 0.0
        self.luu = self.R
        self.lxx = self.Q
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
        self.lx = np.dot(self.Q,(X-Xdes))
        self.lu = np.dot(self.R,U)
        return self.lx,self.lxx,self.lu,self.luu,self.lxu,self.lux

    def computeFinalCostDeriv(self,X,Xdes):
        self.lx = np.dot(self.Q,(X-Xdes))
        return self.lx,self.lxx