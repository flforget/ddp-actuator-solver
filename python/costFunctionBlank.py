import numpy as np
from tools import nRoot

class BlankCostFunction:
    def __init__(self):
        '''
        attributes here
        '''

    def computeCostValue(self,T,XList,Xdes,UList):
        '''
        optional method, useful for debug
        '''
        return cost

    def computeAllCostDeriv(self,X,Xdes,U):
        '''
        code here
        '''
        return lx,lxx,lu,luu,lux,lxu

    def computeFinalCostDeriv(self,X,Xdes):
        '''
        code here
        '''
        return lx,lxx
