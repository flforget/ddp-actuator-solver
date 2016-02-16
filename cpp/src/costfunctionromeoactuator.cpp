#include "costfunctionromeoactuator.h"

CostFunctionRomeoActuator::CostFunctionRomeoActuator()
{
    Q << 100.0,0.0,0.0,0.0,
                0.0,0.0,0.0,0.0,
                0.0,0.0,0.0,0.0,
                0.0,0.0,0.0,0.0;
    R << 0.1;

    lxx = Q;
    luu = R;
    lux << 0.0,0.0,0.0,0.0;
    lxu << 0.0,0.0,0.0,0.0;
    lx.setZero();
}

void CostFunctionRomeoActuator::computeAllCostDeriv(const stateVec_t& X, const commandVec_t& U)
{
//    lx = Q*(X-Xdes);
    lx(0,0) = 100.0*X(0,0);
    lu = R*U;
}

void CostFunctionRomeoActuator::computeFinalCostDeriv(const stateVec_t& X)
{
//    lx = Q*(X-Xdes);
    lx(0,0) = 100.0*X(0,0);
}

// accessors //
stateVec_t& CostFunctionRomeoActuator::getlx()
{
    return lx;
}

stateMat_t& CostFunctionRomeoActuator::getlxx()
{
    return lxx;
}

commandVec_t& CostFunctionRomeoActuator::getlu()
{
    return lu;
}

commandMat_t& CostFunctionRomeoActuator::getluu()
{
    return luu;
}

stateR_commandC_t& CostFunctionRomeoActuator::getlxu()
{
    return lxu;
}

commandR_stateC_t& CostFunctionRomeoActuator::getlux()
{
    return lux;
}
