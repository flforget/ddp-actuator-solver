#include "ilqrsolver.h"

using namespace Eigen;

ILQRSolver::ILQRSolver(DynamicModel& myDynamicModel, CostFunction& myCostFunction)
{
    this->dynamicModel = myDynamicModel;
    this->costFunction = myCostFunction;
    this->stateNb = myDynamicModel.getStateNb();
    this->commandNb = myDynamicModel.getCommandNb();
}

void ILQRSolver::initSolver(Eigen::VectorXd myxInit, Eigen::VectorXd myxDes, unsigned int myT,
                       double mydt, unsigned int myiterMax,double mystopCrit)
{
    this->xInit = myxInit;
    this->xDes = myxDes;
    this->T = myT;
    this->dt = mydt;
    this->iterMax = myiterMax;
    this->stopCrit = mystopCrit;

    this->xList = new Eigen::VectorXd[myT+1];
    this->uList = new Eigen::VectorXd[myT];
    this->updatedxList = new Eigen::VectorXd[myT+1];
    this->updateduList = new Eigen::VectorXd[myT];
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
    for(unsigned int i=1;i<this->T;i++)
    {

    }
}

void ILQRSolver::backwardLoop()
{

}

void ILQRSolver::forwardLoop()
{

}
