#ifndef ILQRSOLVER_H
#define ILQRSOLVER_H

#include "dynamicmodel.h"
#include "costfunction.h"
#include <Eigen/Core>

class ILQRSolver
{
public:
    ILQRSolver(DynamicModel myDynamicModel, CostFunction myCostFunction);
private:
protected:
    // attributes //
public:
private:
    DynamicModel dynamicModel;
    CostFunction costFunction;
    unsigned int stateNb;
    unsigned int commandNb;
    Eigen::VectorXf xInit;
    Eigen::VectorXf xDes;
    unsigned int T;
    double dt;
    unsigned int iter;
    unsigned int iterMax;
    double stopCrit;
    double changeAmount;

    Eigen::VectorXf* xList;
    Eigen::VectorXf* uList;
    Eigen::VectorXf* updatedxList;
    Eigen::VectorXf* updateduList;


protected:
    // methods //
public:
    void initSolver(Eigen::VectorXf myxInit, Eigen::VectorXf myxDes, unsigned int myT,
                    double mydt, unsigned int myiterMax,double mystopCrit);
    void solveTrajectory();
    void initTrajectory();
    void backwardLoop();
    void forwardLoop();
private:
protected:

};

#endif // ILQRSOLVER_H
