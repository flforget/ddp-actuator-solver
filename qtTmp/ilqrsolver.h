#ifndef ILQRSOLVER_H
#define ILQRSOLVER_H

#include "config.h"

#include "dynamicmodel.h"
#include "costfunction.h"
#include <Eigen/Core>

using namespace Eigen;

class ILQRSolver
{
public:
    ILQRSolver(DynamicModel& myDynamicModel, CostFunction& myCostFunction);
private:
protected:
    // attributes //
public:
private:
    DynamicModel dynamicModel;
    CostFunction costFunction;
    unsigned int stateNb;
    unsigned int commandNb;
    stateVec_t xInit;
    stateVec_t xDes;
    unsigned int T;
    double dt;
    unsigned int iter;
    unsigned int iterMax;
    double stopCrit;
    double changeAmount;

    stateVec_t* xList;
    commandVec_t* uList;
    stateVec_t* updatedxList;
    commandVec_t* updateduList;


protected:
    // methods //
public:
    void initSolver(stateVec_t myxInit, stateVec_t myxDes, unsigned int myT,
                    double mydt, unsigned int myiterMax,double mystopCrit);
    void solveTrajectory();
    void initTrajectory();
    void backwardLoop();
    void forwardLoop();
private:
protected:

};

#endif // ILQRSOLVER_H
