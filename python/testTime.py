__author__ = 'fforget'

import numpy as np
from iLQRSolverImproved import *
from iLQRSolver import *
from costFunction import *
from dynamicModel import *
from costFunctionImproved import *
from dynamicModelImproved import *
import matplotlib.pyplot as pl
from random import uniform
import time
import cProfile

def testClassic(Xinit,Xdes,N,dt,itermax,stopCrit,iter):
	for i in range(iter):
		solver.solveTrajectory(Xinit,Xdes,N,dt,20,1e-3)
def testImproved(Xinit,Xdes,N,dt,itermax,stopCrit,iter):
	for i in range(iter):
		solverImproved.solveTrajectory(Xinit,Xdes,N,dt,20,1e-3)	
		
def testClassicInit(Xinit,Xdes,N,dt,itermax,stopCrit,iter):
	for i in range(iter):
		solver.initTrajectory()
def testImprovedInit(Xinit,Xdes,N,dt,itermax,stopCrit,iter):
	for i in range(iter):	
		solverImproved.initTrajectory()
		
def testClassicForward(kList,KList,XList,UList,iter):
	for i in range(iter):
		solver.forwardLoop(kList,KList,XList,UList)		
		
def testImprovedForward(iter):
	for i in range(iter):
		solverImproved.forwardLoop()		

def testClassicBackward(XList,UList,iter):
	for i in range(iter):
		solver.backwardLoop(XList,UList)		
		
def testImprovedBackward(iter):
	for i in range(iter):
		solverImproved.backwardLoop()
				
Xinit = np.matrix([ [uniform(-10,10)],
                    [uniform(-1.0,1.0)],
                    [uniform(-10,10)],
                    [uniform(-1.0,1.0)]])
Xdes = np.matrix([  [uniform(-10,10)],
                    [0.0],
                    [0.0],
                    [0.0]])


N = 50
dt = 1e-4


model = SimpleRomeoActuatorDynamicModel()
costFunction = CostFunctionSimpleRomeoActuator()
modelImproved = SimpleRomeoActuatorDynamicModelImproved()
costFunctionImproved = CostFunctionSimpleRomeoActuatorImproved()


solver = ILQRSolver(model,costFunction)
solverImproved = ILQRSolverImproved(modelImproved,costFunctionImproved)

XList,UList = solver.solveTrajectory(Xinit,Xdes,N,dt,20,1e-3)
kList,KList = solver.backwardLoop(XList,UList)
solverImproved.solveTrajectory(Xinit,Xdes,N,dt,20,1e-3)

cProfile.run('testClassic(Xinit,Xdes,N,dt,20,1e-3,10)')
cProfile.run('testImproved(Xinit,Xdes,N,dt,20,1e-3,10)')
#~ cProfile.run('testClassicInit(Xinit,Xdes,N,dt,20,1e-3,100)')
#~ cProfile.run('testImprovedInit(Xinit,Xdes,N,dt,20,1e-3,100)')
#~ cProfile.run('testClassicForward(kList,KList,XList,UList,100)')
#~ cProfile.run('testImprovedForward(100)')
#~ cProfile.run('testClassicBackward(XList,UList,50)')
#~ cProfile.run('testImprovedBackward(50)')
