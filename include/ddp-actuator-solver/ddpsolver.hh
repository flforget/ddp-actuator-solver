#ifndef DDPSOLVER_H
#define DDPSOLVER_H

#include "dynamicmodel.hh"
#include "costfunction.hh"

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <qpOASES.hpp>
#include <qpOASES/QProblemB.hpp>

#define ENABLE_QPBOX 1
#define DISABLE_QPBOX 0
#define ENABLE_FULLDDP 1
#define DISABLE_FULLDDP 0

USING_NAMESPACE_QPOASES

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::MatrixXd)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::VectorXd)

template<typename precision, int stateSize, int commandSize>
class DDPSolver
{
public:
  typedef Eigen::Matrix<precision, stateSize, 1> stateVec_t;  // 1 x stateSize
  typedef Eigen::Matrix<precision, 1, stateSize> stateVecTrans_t; // 1 x stateSize
  typedef Eigen::Matrix<precision, stateSize, stateSize>stateMat_t; // stateSize x stateSize
  typedef Eigen::Matrix<precision, stateSize, stateSize>stateTens_t[stateSize]; // stateSize x stateSize x stateSize

  // typedef for commandSize types
  typedef Eigen::Matrix<precision, commandSize, 1>commandVec_t; // commandSize x 1
  typedef Eigen::Matrix<precision, 1, commandSize>commandVecTrans_t; // 1 x commandSize
  typedef Eigen::Matrix<precision, commandSize, commandSize>commandMat_t; // commandSize x commandSize
  typedef Eigen::Matrix<precision, commandSize, commandSize>
  commandTens_t[commandSize]; // stateSize x commandSize x commandSize

  // typedef for mixed stateSize and commandSize types
  typedef Eigen::Matrix<precision, stateSize, commandSize>stateR_commandC_t; // stateSize x commandSize
  typedef Eigen::Matrix<precision, stateSize, commandSize>stateR_commandC_stateD_t[stateSize]; // stateSize x commandSize x stateSize
  typedef Eigen::Matrix<precision, stateSize, commandSize>stateR_commandC_commandD_t[commandSize]; // stateSize x commandSize x commandSize
  typedef Eigen::Matrix<precision, commandSize, stateSize>commandR_stateC_t; // commandSize x stateSize
  typedef Eigen::Matrix<precision, commandSize, stateSize>commandR_stateC_stateD_t[stateSize]; // commandSize x stateSize x stateSize
  typedef Eigen::Matrix<precision, commandSize, stateSize>commandR_stateC_commandD_t[commandSize]; // commandSize x stateSize x commandSize
  typedef Eigen::Matrix<precision, stateSize, stateSize>stateR_stateC_commandD_t[commandSize]; // stateSize x stateSize x commandSize
  typedef Eigen::Matrix<precision, commandSize, commandSize>commandR_commandC_stateD_t[stateSize]; // commandSize x commandSize x stateSize

  typedef std::vector<stateVec_t> stateVecTab_t;
  typedef std::vector<commandVec_t> commandVecTab_t;
  typedef std::vector<commandR_stateC_t> commandR_stateC_tab_t;

  typedef DynamicModel<precision, stateSize, commandSize> DynamicModel_t;
  typedef CostFunction<precision, stateSize, commandSize> CostFunction_t;

public:
  struct traj
  {
    stateVecTab_t xList;
    commandVecTab_t uList;
    unsigned int iter;
  };

public:
private:
protected:
  // attributes //
public:
private:
  DynamicModel_t * dynamicModel;
  CostFunction_t * costFunction;
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
  commandVec_t zeroCommand;

  stateVecTab_t xList;
  commandVecTab_t uList;
  stateVecTab_t updatedxList;
  commandVecTab_t updateduList;
  stateVecTab_t tmpxPtr;
  commandVecTab_t tmpuPtr;
  struct traj lastTraj;

  stateVec_t nextVx;
  stateMat_t nextVxx;
  stateVec_t Qx;
  stateMat_t Qxx;
  commandVec_t Qu;
  commandMat_t Quu, Quu_reg;
  Eigen::LLT<commandMat_t> lltofQuu;
  commandMat_t QuuInv;
  commandR_stateC_t Qux, Qux_reg;
  commandVec_t k;
  commandR_stateC_t K;
  commandVecTab_t kList;
  commandR_stateC_tab_t KList;
  double alphaList[5];
  double alpha;

  double mu;
  stateMat_t muEye;
  unsigned char completeBackwardFlag;

  /* QP variables */
  QProblemB* qp;
  bool enableQPBox;
  bool enableFullDDP;
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
  DDPSolver(DynamicModel_t& myDynamicModel, CostFunction_t& myCostFunction,
            bool fullDDP = 0, bool QPBox = 0)
  {
    dynamicModel = &myDynamicModel;
    costFunction = &myCostFunction;
    stateNb = myDynamicModel.getStateNb();
    commandNb = myDynamicModel.getCommandNb();
    enableQPBox = QPBox;
    enableFullDDP = fullDDP;
    zeroCommand.setZero();
    if (QPBox)
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

  void FirstInitSolver(stateVec_t& myxInit, stateVec_t& myxDes,
                       unsigned int& myT, double& mydt, unsigned int& myiterMax,
                       double& mystopCrit)
  {
    xInit = myxInit;
    xDes = myxDes;
    T = myT;
    dt = mydt;
    iterMax = myiterMax;
    stopCrit = mystopCrit;

    xList.resize(myT + 1);
    uList.resize(myT);
    updatedxList.resize(myT + 1);
    updateduList.resize(myT);
    tmpxPtr.resize(myT + 1);
    tmpuPtr.resize(myT);
    k.setZero();
    K.setZero();
    kList.resize(myT);
    KList.resize(myT);

    alphaList[0] = 1.0;
    alphaList[1] = 0.8;
    alphaList[2] = 0.6;
    alphaList[3] = 0.4;
    alphaList[4] = 0.2;
    alpha = 1.0;
  }

  void initSolver(stateVec_t& myxInit, stateVec_t& myxDes)
  {
    xInit = myxInit;
    xDes = myxDes;
  }

  void solveTrajectory()
  {
    initTrajectory();
    for (iter = 1; iter < iterMax; iter++)
    {
      backwardLoop();
      forwardLoop();
      if (changeAmount < stopCrit)
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

  void initTrajectory()
  {
    xList[0] = xInit;
    for (unsigned int i = 0; i < T; i++)
    {
      uList[i] = zeroCommand;
      xList[i + 1] = dynamicModel->computeNextState(dt, xList[i],
                     zeroCommand);
    }
  }

  void backwardLoop()
  {
    costFunction->computeFinalCostDeriv(xList[T], xDes);
    nextVx = costFunction->getlx();
    nextVxx = costFunction->getlxx();

    mu = 0.0;
    completeBackwardFlag = 0;

    while (!completeBackwardFlag)
    {
      completeBackwardFlag = 1;
      muEye = stateMat_t::Constant(mu);
      for (int i = T - 1; i >= 0; i--)
      {
        x = xList[i];
        u = uList[i];

        dynamicModel->computeAllModelDeriv(dt, x, u);
        costFunction->computeAllCostDeriv(x, xDes, u);

        Qx = costFunction->getlx()
             + dynamicModel->getfx().transpose() * nextVx;
        Qu = costFunction->getlu()
             + dynamicModel->getfu().transpose() * nextVx;
        Qxx = costFunction->getlxx()
              + dynamicModel->getfx().transpose() * (nextVxx)
              * dynamicModel->getfx();
        Quu = costFunction->getluu()
              + dynamicModel->getfu().transpose() * (nextVxx)
              * dynamicModel->getfu();
        Qux = costFunction->getlux()
              + dynamicModel->getfu().transpose() * (nextVxx)
              * dynamicModel->getfx();
        Quu_reg = costFunction->getluu()
                  + dynamicModel->getfu().transpose() * (nextVxx + muEye)
                  * dynamicModel->getfu();
        Qux_reg = costFunction->getlux()
                  + dynamicModel->getfu().transpose() * (nextVxx + muEye)
                  * dynamicModel->getfx();

        if (enableFullDDP)
        {
          Qxx += dynamicModel->computeTensorContxx(nextVx);
          Qux += dynamicModel->computeTensorContux(nextVx);
          Quu += dynamicModel->computeTensorContuu(nextVx);
          Qux_reg += dynamicModel->computeTensorContux(nextVx);
          Quu_reg += dynamicModel->computeTensorContuu(nextVx);
        }

        if (!isQuudefinitePositive(Quu_reg))
        {

          std::cout << "regularization" << std::endl; // to remove
          if (mu == 0.0)
            mu += 1e-4;
          else
            mu *= 10;
          completeBackwardFlag = 0;
          break;
        }

        QuuInv = Quu.inverse();

        if (enableQPBox)
        {
          nWSR = 10;
          H = Quu_reg;
          g = Qu;
          lb = lowerCommandBounds - u;
          ub = upperCommandBounds - u;
          qp->init(H.data(), g.data(), lb.data(), ub.data(), nWSR);
          qp->getPrimalSolution(xOpt);
          k = Eigen::Map<commandVec_t>(xOpt);
          K = -QuuInv * Qux;
          for (unsigned int i_cmd = 0; i_cmd < commandNb; i_cmd++)
          {
            if ((k[i_cmd] == lowerCommandBounds[i_cmd])
                | (k[i_cmd] == upperCommandBounds[i_cmd]))
            {
              K.row(i_cmd).setZero();
            }
          }
        }
        else
        {
          k = -QuuInv * Qu;
          K = -QuuInv * Qux;
        }

        /*nextVx = Qx - K.transpose()*Quu*k;
         nextVxx = Qxx - K.transpose()*Quu*K;*/
        nextVx = Qx + K.transpose() * Quu * k + K.transpose() * Qu
                 + Qux.transpose() * k;
        nextVxx = Qxx + K.transpose() * Quu * K + K.transpose() * Qux
                  + Qux.transpose() * K;
        nextVxx = 0.5 * (nextVxx + nextVxx.transpose());

        kList[i] = k;
        KList[i] = K;
      }
    }
  }

  void forwardLoop()
  {
    changeAmount = 0.0;
    updatedxList[0] = xInit;
    // Line search to be implemented
    alpha = 1.0;
    for (unsigned int i = 0; i < T; i++)
    {
      updateduList[i] = uList[i] + alpha * kList[i]
                        + KList[i] * (updatedxList[i] - xList[i]);
      updatedxList[i + 1] = dynamicModel->computeNextState(dt,
                            updatedxList[i], updateduList[i]);
      for (unsigned int j = 0; j < commandNb; j++)
      {
        updateduList[i] = uList[i] + alpha * kList[i] + KList[i] *
                          (updatedxList[i] - xList[i]);
        updatedxList[i + 1] = dynamicModel->computeNextState(dt, updatedxList[i],
                              updateduList[i]);
        for (unsigned int j = 0; j < commandNb; j++)
        {
          changeAmount += fabs(uList[i](j, 0) - updateduList[i](j, 0));
        }
      }
    }
  }

  DDPSolver::traj getLastSolvedTrajectory()
  {
    lastTraj.xList = updatedxList;
    lastTraj.uList = updateduList;
    lastTraj.iter = iter;
    return lastTraj;
  }

  bool isQuudefinitePositive(const commandMat_t & Quu_reg)
  {
    lltofQuu.compute(Quu_reg);
    if (lltofQuu.info() == Eigen::NumericalIssue)
    {
      std::cout << "not sdp" << std::endl;
      return false;
    }
    return true;
  }
protected:

};

#endif // DDPSOLVER_H
