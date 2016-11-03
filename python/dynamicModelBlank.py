import numpy as np
from random import uniform

class BlankDynamicModel:
    """ discrete dynamic model definition class
            _ return derivatives
            _ return nextState from currentState
            _ input of iLQRSolver
    """
    def __init__(self): # constructor
        '''
        attributes here
        '''
        self.stateNumber = 4
        self.commandNumber = 1

    def computeNextState(self,dt,X,U): # compute state for next time step
        '''
        code here
        '''
        return nextX


    def computeAllModelDeriv(self,dt,X,U):
        '''
        code here
        '''
        return fx,fu,fxx,fuu,fux,fxu
