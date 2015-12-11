#include "costfunctionpneumaticarmelbow.h"

CostFunctionPneumaticarmElbow::CostFunctionPneumaticarmElbow()

{
      Q << 5e1*1.0, 0.0, 0.0, 0.0,
      0.0,  0.0,    0.0,    0.0,
      1.75e1*1.0,  0.0,    1e-4*1.0,    0.0,
      1e1*0.0,  0.0,    0.0,    1e-4*1.0;
   /* Qf << 1e5*1.0, 0.0, 0.0, 0.0,
      0.0,  0.0,    0.0,    0.0,
      0.0,  0.0,   1e-5*1.0,    0.0,
      0.0,  0.0,    0.0,    1e-5*1.0;*/

   /* Q << 1e4*1.0, 0.0,
        0.0, 0.0;*/   
    Qf = Q;

    R << 0.0000065, 0.0,
      0.0,  0.0000065;

    lxx = Q;
    luu = R;
    //lux << 0.0,0.0,0.0;
    //lxu << 0.0,0.0,0.0;
    lx.setZero();
}

void CostFunctionPneumaticarmElbow::computeAllCostDeriv(const stateVec_t& X, const stateVec_t& Xdes, const commandVec_t& U)
{
    lx = Q*(X-Xdes);
    //lx(0,0) = 1e-3*8*(X(0,0)-Xdes(0,0));
    lu = R*U;
}

void CostFunctionPneumaticarmElbow::computeFinalCostDeriv(const stateVec_t& X, const stateVec_t& Xdes)
{
    lx = Qf*(X-Xdes);
    //lx(0,0) = 1e-3*1.0*(X(0,0)-Xdes(0,0));
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
