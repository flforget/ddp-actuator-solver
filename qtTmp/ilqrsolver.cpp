#include "ilqrsolver.h"

using namespace Eigen;

ILQRSolver::ILQRSolver(DynamicModel& myDynamicModel, CostFunction& myCostFunction)
{
    this->dynamicModel = myDynamicModel;
    this->costFunction = myCostFunction;
    this->stateNb = myDynamicModel.getStateNb();
    this->commandNb = myDynamicModel.getCommandNb();
}

void ILQRSolver::initSolver(stateVec_t myxInit, stateVec_t myxDes, unsigned int myT,
                       double mydt, unsigned int myiterMax,double mystopCrit)
{
    this->xInit = myxInit;
    this->xDes = myxDes;
    this->T = myT;
    this->dt = mydt;
    this->iterMax = myiterMax;
    this->stopCrit = mystopCrit;

    this->xList = new stateVec_t[myT+1];
    this->uList = new commandVec_t[myT];
    this->updatedxList = new stateVec_t[myT+1];
    this->updateduList = new commandVec_t[myT];
}

void ILQRSolver::solveTrajectory()
{
    this->initTrajectory();
    for(this->iter=0;this->iter<this->iterMax;this->iter++)
    {
        this->backwardLoop();
        this->forwardLoop();
        this->xList = this->updatedxList;
        this->uList = this->updateduList;
        if(this->changeAmount<this->stopCrit)
            break;
    }
}

void ILQRSolver::initTrajectory()
{
    this->xList[0] = this->xInit;
    commandVec_t zeroCommand;
    zeroCommand << commandVec_t::Zero(commandSize,1);
    for(unsigned int i=0;i<this->T;i++)
    {
        this->uList[i] = zeroCommand;
        this->xList[i+1] = this->dynamicModel.computeNextState(this->dt,this->xList[i],zeroCommand);
    }
}

void ILQRSolver::backwardLoop()
{

}

void ILQRSolver::forwardLoop()
{

}
