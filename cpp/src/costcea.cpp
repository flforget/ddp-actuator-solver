#include "costcea.h"

CostCea::CostCea()
{
    Q <<    1000.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0;
    R <<    0.1,0.0,
            0.0,0.00001;

    lxx = Q;
    luu = R;
    lux << 0.0,0.0,0.0;
    lxu << 0.0,0.0,0.0;
    lx.setZero();
}

void CostCea::computeAllCostDeriv(const stateVec_t& X, const commandVec_t& U)
{
//    lx = Q*(X-Xdes);
    lx = Q*X;
    lu = R*U;
}

void CostCea::computeFinalCostDeriv(const stateVec_t& X)
{
    lx = Q*X;
}

