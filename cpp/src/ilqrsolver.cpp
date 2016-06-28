#include "ilqrsolver.h"

/* Debug */
#include <iostream>
using namespace std;
/* */

using namespace Eigen;

ILQRSolver::ILQRSolver(DynamicModel& myDynamicModel, CostFunction& myCostFunction,bool fullDDP,bool QPBox)
{
    dynamicModel = &myDynamicModel;
    costFunction = &myCostFunction;
    stateNb = myDynamicModel.getStateNb();
    commandNb = myDynamicModel.getCommandNb();
    enableQPBox = QPBox;
    enableFullDDP = fullDDP;
    if(QPBox)
    {
        qp = new QProblemB(commandNb);
        Options myOptions;
        myOptions.printLevel = PL_LOW;
        myOptions.enableRegularisation = BT_TRUE;
        myOptions.initialStatusBounds = ST_INACTIVE;
        myOptions.numRefinementSteps = 1;
        myOptions.enableCholeskyRefactorisation = 1;
        qp->setOptions(myOptions);

        xOpt = new real_t[commandNb];
        lowerCommandBounds = myDynamicModel.getLowerCommandBounds();
        upperCommandBounds = myDynamicModel.getUpperCommandBounds();
    }
}

void ILQRSolver::FirstInitSolver(stateVec_t& myxInit, stateVec_t& myxDes, unsigned int& myT,
                       double& mydt, unsigned int& myiterMax,double& mystopCrit)
{
    xInit = myxInit-myxDes;
    xDes = myxDes;
    T = myT;
    dt = mydt;
    iterMax = myiterMax;
    stopCrit = mystopCrit;

    xList.resize(myT+1);
    uList.resize(myT);
    updatedxList.resize(myT+1);
    updateduList.resize(myT);
    tmpxPtr.resize(myT+1);
    tmpuPtr.resize(myT);
    k.setZero();
    K.setZero();
    kList.resize(myT);
    KList.resize(myT);

    /*xList = new stateVec_t[myT+1];
    uList = new commandVec_t[myT];
    updatedxList = new stateVec_t[myT+1];
    updateduList = new commandVec_t[myT];
    k.setZero();
    K.setZero();
    kList = new commandVec_t[myT];
    KList = new commandR_stateC_t[myT];*/

    alphaList[0] = 1.0;
    alphaList[1] = 0.8;
    alphaList[2] = 0.6;
    alphaList[3] = 0.4;
    alphaList[4] = 0.2;
    alpha = 1.0;
}

void ILQRSolver::initSolver(stateVec_t& myxInit, stateVec_t& myxDes)
{
    xInit = myxInit - myxDes;
    xDes = myxDes;
}

void ILQRSolver::solveTrajectory()
{
    initTrajectory();
    for(iter=0;iter<iterMax;iter++)
    {
        backwardLoop();
        forwardLoop();
        if(changeAmount<stopCrit)
        {
          break;
        }
        tmpxPtr = xList;
        tmpuPtr = uList;
        xList = updatedxList;
        updatedxList = tmpxPtr;
        uList = updateduList;
        updateduList = tmpuPtr;
    }
}

void ILQRSolver::initTrajectory()
{
    xList[0] = xInit;
    commandVec_t zeroCommand;
    stateVec_t Xe;
    Xe = xInit;
    zeroCommand.setZero();
    for(unsigned int i=0;i<T;i++)
    {
        uList[i] = zeroCommand;
        xList[i+1] = dynamicModel->computeNextState(dt,xList[i] ,xDes,zeroCommand) - xDes;
        //Xe = xList[i] - xDes;
    }
}

void ILQRSolver::backwardLoop()
{
    costFunction->computeFinalCostDeriv(xList[T]);
    nextVx = costFunction->getlx();
    nextVxx = costFunction->getlxx();

    mu = 0.0;
    completeBackwardFlag = 0;

    while(!completeBackwardFlag)
    {
        completeBackwardFlag = 1;
        muEye = mu*stateMat_t::Zero();
        for(int i=T-1;i>=0;i--)
        {
            x = xList[i];
            u = uList[i];

            dynamicModel->computeAllModelDeriv(dt,x,xDes,u);
            costFunction->computeAllCostDeriv(x,u);

            Qx = costFunction->getlx() + dynamicModel->getfx().transpose() * nextVx;
            Qu = costFunction->getlu() + dynamicModel->getfu().transpose() * nextVx;
            Qxx = costFunction->getlxx() + dynamicModel->getfx().transpose() * (nextVxx+muEye) * dynamicModel->getfx();
            Quu = costFunction->getluu() + dynamicModel->getfu().transpose() * (nextVxx+muEye) * dynamicModel->getfu();
            Qux = costFunction->getlux() + dynamicModel->getfu().transpose() * (nextVxx+muEye) * dynamicModel->getfx();

            if(enableFullDDP)
            {
                Qxx += dynamicModel->computeTensorContxx(nextVx);
                Qux += dynamicModel->computeTensorContux(nextVx);
                Quu += dynamicModel->computeTensorContuu(nextVx);
            }

            QuuInv = Quu.inverse();

            if(!isQuudefinitePositive(Quu))
            {
                /*
                  To be Implemented : Regularization (is Quu definite positive ?)
                */
                if(mu==0.0) mu += 1e-4;
                else mu *= 10;
                completeBackwardFlag = 0;
                break;
            }

            if(enableQPBox)
            {
                nWSR = 10;
                H = Quu;
                g = Qu;
                lb = lowerCommandBounds - u;
                ub = upperCommandBounds - u;
                qp->init(H.data(),g.data(),lb.data(),ub.data(),nWSR);
                qp->getPrimalSolution(xOpt);
                k = Map<commandVec_t>(xOpt);
                K = -QuuInv*Qux;
                for(unsigned int i_cmd=0;i_cmd<commandNb;i_cmd++)
                {
                    if((k[i_cmd] == lowerCommandBounds[i_cmd]) | (k[i_cmd] == upperCommandBounds[i_cmd]))
                    {
                        K.row(i_cmd).setZero();
                    }
                }
            }
            else
            {
                k = -QuuInv*Qu;
                K = -QuuInv*Qux;
            }

            /*nextVx = Qx - K.transpose()*Quu*k;
            nextVxx = Qxx - K.transpose()*Quu*K;*/
            nextVx = Qx + K.transpose()*Quu*k + K.transpose()*Qu + Qux.transpose()*k;
            nextVxx = Qxx + K.transpose()*Quu*K+ K.transpose()*Qux + Qux.transpose()*K;
            nextVxx = 0.5*(nextVxx + nextVxx.transpose());

            kList[i] = k;
            KList[i] = K;
        }
    }
}

void ILQRSolver::forwardLoop()
{
    changeAmount = 0.0;
    updatedxList[0] = xInit;
    // Line search to be implemented
    alpha = 1.0;
    for(unsigned int i=0;i<T;i++)
    {
        updateduList[i] = uList[i] + alpha*kList[i] + KList[i]*(updatedxList[i] - xList[i]);
        updatedxList[i+1] = dynamicModel->computeNextState(dt,updatedxList[i],xDes,updateduList[i]) - xDes;
        for(unsigned int j=0;j<commandNb;j++)
        {
            changeAmount += abs(uList[i](j,0) - updateduList[i](j,0));
        }
    }
}

ILQRSolver::traj ILQRSolver::getLastSolvedTrajectory()
{
    lastTraj.xList = updatedxList;
    for(int i=0;i<T+1;i++)lastTraj.xList[i] += xDes;
    lastTraj.uList = updateduList;
    for(int i=0;i<T;i++) lastTraj.uList[i] += dynamicModel->commandOffset;
    lastTraj.iter = iter;
    return lastTraj;
}

bool ILQRSolver::isQuudefinitePositive(const commandMat_t & Quu)
{
    /*
      Todo : check if Quu is definite positive
    */
    //Eigen::JacobiSVD<commandMat_t> svd_Quu (Quu, ComputeThinU | ComputeThinV);
    Eigen::VectorXcd singular_values = Quu.eigenvalues();

    for(long i = 0; i < Quu.cols(); ++i)
    {
        if (singular_values[i].real() < 0.)
        {
            std::cout << "not sdp" << std::endl;
            return false;
        }
    }
    return true;
}
