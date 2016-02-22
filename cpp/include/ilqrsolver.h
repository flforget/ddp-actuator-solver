#ifndef ILQRSOLVER_H
#define ILQRSOLVER_H

#include "config.h"

#include "dynamicmodel.h"
#include "costfunction.h"
#include <Eigen/Dense>
#include <qpOASES.hpp>
#include <qpOASES/QProblemB.hpp>

#define ENABLE_QPBOX 1
#define DISABLE_QPBOX 0

using namespace Eigen;
USING_NAMESPACE_QPOASES

class ILQRSolver
{
public:
    struct traj
    {
        stateVec_t* xList;
        commandVec_t* uList;
        unsigned int iter;
    };

public:
    ILQRSolver(DynamicModel& myDynamicModel, CostFunction& myCostFunction,bool QPBox=0);
private:
protected:
    // attributes //
public:
private:
    DynamicModel* dynamicModel;
    CostFunction* costFunction;
    unsigned int stateNb;
    unsigned int commandNb;
    stateVec_t x;
    commandVec_t u;
    stateVec_t xInit;
    stateVec_t xDes;
    unsigned int T;
    unsigned int iter;
    double dt;
    unsigned int iterMax;
    double stopCrit;
    double changeAmount;

    stateVec_t* xList;
    commandVec_t* uList;
    stateVec_t* updatedxList;
    commandVec_t* updateduList;
    struct traj lastTraj;

    stateVec_t nextVx;
    stateMat_t nextVxx;
    stateVec_t Qx;
    stateMat_t Qxx;
    commandVec_t Qu;
    commandMat_t Quu;
    commandMat_t QuuInv;
    commandR_stateC_t Qux;
    commandVec_t k;
    commandR_stateC_t K;
    commandVec_t* kList;
    commandR_stateC_t* KList;
    double alphaList[5];
    double alpha;



    double mu;
    stateMat_t muEye;
    unsigned char completeBackwardFlag;

    /* QP variables */
    QProblemB* qp;
    bool enableQPBox;
    commandMat_t H;
    commandVec_t g;
    commandVec_t lowerCommandBounds;
    commandVec_t upperCommandBounds;
    commandVec_t lb;
    commandVec_t ub;
    int nWSR;
    real_t* xOpt;
protected:
    // methods //
public:
    void FirstInitSolver(stateVec_t& myxInit, stateVec_t& myxDes, unsigned int& myT,
                    double& mydt, unsigned int& myiterMax,double& mystopCrit);
    void initSolver(stateVec_t& myxInit, stateVec_t& myxDes);
    void solveTrajectory();
    struct traj getLastSolvedTrajectory();
//private:
    void initTrajectory();
    void backwardLoop();
    void forwardLoop();
    bool isQuudefinitePositive(const commandMat_t & Quu); 
protected:

};

#endif // ILQRSOLVER_H
