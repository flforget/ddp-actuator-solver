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
    lx = Q*X;
    lu = R*U;
}

void CostFunctionRomeoActuator::computeFinalCostDeriv(const stateVec_t& X)
{
    lx = Q*X;
}

