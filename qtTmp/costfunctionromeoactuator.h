#ifndef COSTFUNCTIONROMEOACTUATOR_H
#define COSTFUNCTIONROMEOACTUATOR_H

#include "config.h"

#include "costfunction.h"

#include <Eigen/Core>

using namespace Eigen;

class CostFunctionRomeoActuator : public CostFunction
{
public:
    CostFunctionRomeoActuator();
private:
    stateMat_t Q;
    commandMat_t R;
    stateVec_t lx;
    stateMat_t lxx;
    commandVec_t lu;
    commandMat_t luu;
    commandR_stateC_t lux;
    stateR_commandC_t lxu;
protected:
    // attributes //
public:
private:

protected:
    // methods //
public:
    void computeAllCostDeriv(stateVec_t& X, stateVec_t& Xdes, commandVec_t& U);
    void commuteFinalCostDeriv(stateVec_t& X, stateVec_t& Xdes);
private:
protected:
    // accessors //
public:
    stateVec_t getlx();
    stateMat_t getlxx();
    commandVec_t getlu();
    commandMat_t getluu();
    commandR_stateC_t getlux();
    stateR_commandC_t getlxu();
};

#endif // COSTFUNCTIONROMEOACTUATOR_H