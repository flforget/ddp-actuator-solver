#ifndef ILQRSOLVER_H
#define ILQRSOLVER_H

#include "dynamicmodel.h"
#include "costfunction.h"
#include <Eigen/Core>

using namespace Eigen;

#ifdef stateSize
typedef Matrix<double,stateSize,1> stateVec;
#else
typedef Matrix<double,Dynamic,1> stateVec;
#endif
#ifdef commandSize
typedef Matrix<double,commandSize,1> commandVec;
#else
typedef Matrix<double,Dynamic,1> commandVec;
#endif

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
    stateVec xInit;
    stateVec xDes;
    unsigned int T;
    double dt;
    unsigned int iter;
    unsigned int iterMax;
    double stopCrit;
    double changeAmount;

    stateVec* xList;
    stateVec* uList;
    stateVec* updatedxList;
    stateVec* updateduList;


protected:
    // methods //
public:
    void initSolver(stateVec myxInit, stateVec myxDes, unsigned int myT,
                    double mydt, unsigned int myiterMax,double mystopCrit);
    void solveTrajectory();
    void initTrajectory();
    void backwardLoop();
    void forwardLoop();
private:
protected:

};

#endif // ILQRSOLVER_H
