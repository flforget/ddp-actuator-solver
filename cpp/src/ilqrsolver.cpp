#include "ilqrsolver.h"

ILQRSolver::ILQRSolver(DynamicModel myDynamicModel, CostFunction myCostFunction)
{
    this->dynamicModel = myDynamicModel;
    this->costFunction = myCostFunction;
    this->stateNb = myDynamicModel.getStateNb();
    this->commandNb = myDynamicModel.getCommandNb();
}

void ILQRSolver::initSolver(Eigen::VectorXf myxInit, Eigen::VectorXf myxDes, unsigned int myT,
                       double mydt, unsigned int myiterMax,double mystopCrit)
{
    this->xInit = myxInit;
    this->xDes = myxDes;
    this->T = myT;
    this->dt = mydt;
    this->iterMax = myiterMax;
    this->stopCrit = mystopCrit;

    this->xList = new Eigen::VectorXf[myT+1];
    this->uList = new Eigen::VectorXf[myT];
    this->updatedxList = new Eigen::VectorXf[myT+1];
    this->updateduList = new Eigen::VectorXf[myT];

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
    Eigen::Vector<double,this->commandNb> zerosCommand;
    for(int i=0;i<this->T;i++)
    {
        uList[i] = zerosCommand;
    }
}

void ILQRSolver::backwardLoop()
{

}

void ILQRSolver::forwardLoop()
{

}
