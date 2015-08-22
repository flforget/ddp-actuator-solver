#include "costfunctionpneumaticarmelbow.h"

CostFunctionPneumaticarmElbow::CostFunctionPneumaticarmElbow()

{
    Q << 50.0,0.0,0.0,
                0.0,0.0,0.0,
                0.0,0.0,0.0;
    R << 0.0001;

    lxx = Q;
    luu = R;
    lux << 0.0,0.0,0.0;
    lxu << 0.0,0.0,0.0;
    lx.setZero();
}

void CostFunctionPneumaticarmElbow::computeAllCostDeriv(const stateVec_t& X, const stateVec_t& Xdes, const commandVec_t& U)
{
//    lx = Q*(X-Xdes);
    lx(0,0) = 50.0*(X(0,0)-Xdes(0,0));
    lu = R*U;
}

void CostFunctionPneumaticarmElbow::computeFinalCostDeriv(const stateVec_t& X, const stateVec_t& Xdes)
{
//    lx = Q*(X-Xdes);
    lx(0,0) = 50.0*(X(0,0)-Xdes(0,0));
}

// accessors //
stateVec_t CostFunctionPneumaticarmElbow::getlx()
{
    return lx;
}

stateMat_t CostFunctionPneumaticarmElbow::getlxx()
{
    return lxx;
}

commandVec_t CostFunctionPneumaticarmElbow::getlu()
{
    return lu;
}

commandMat_t CostFunctionPneumaticarmElbow::getluu()
{
    return luu;
}

stateR_commandC_t CostFunctionPneumaticarmElbow::getlxu()
{
    return lxu;
}

commandR_stateC_t CostFunctionPneumaticarmElbow::getlux()
{
    return lux;
}
