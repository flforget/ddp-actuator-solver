#include "costfunctionpneumaticarmelbow.h"

CostFunctionPneumaticarmElbow::CostFunctionPneumaticarmElbow()

{
    Q <<1e-1*1, 0.0, 0.0, 0.0, 0,0,0,0, 
        0.0,1e-1*1, 0.0, 0.0, 0,0,0,0,
        2.0e-2*0.0, 0.0, 1e-5*0.0, 0.0, 0,0,0,0,
         0.0, 0.0, 0.0, 1e-5*0.0, 0,0,0,0,
         0,0,0,0, 0,0,0,0,
         0,0,0,0, 0,0,0,0,
         0,0,0,0, 0,0,0,0,
         0,0,0,0, 0,0,0,0;
   /* Qf <<8e4*1.0, 0.0, 0.0, 0.0, 
        0.0,1e-3*0.0, 0.0, 0.0,
        0.0, 0.0, 5e-5*1.0, 0.0,
         0.0, 0.0, 0.0, 5e-5*1.0;*/
    /*Qf <<1e0*5, 0.0, 0.0, 0.0, 0,0,0,0, 
        0.0,1e0*8, 0.0, 0.0, 0,0,0,0,
        2.0e-2*0.0, 0.0, 1e-5*0.0, 0.0, 0,0,0,0,
         0.0, 0.0, 0.0, 1e-5*0.0, 0,0,0,0,
         0,0,0,0, 0,0,0,0,
         0,0,0,0, 0,0,0,0,
         0,0,0,0, 0,0,0,0,
         0,0,0,0, 0,0,0,0;*/
    Qf = Q; Qf(0,0) = 0; Qf(1,1) = 0;
    R << 5e-3,0,
            0, 5e-3;
    lxx = Q;
    luu = R;
    //lux << 0.0,0.0;
    //lxu << 0.0,0.0;
    lx.setZero();
}

void CostFunctionPneumaticarmElbow::computeAllCostDeriv(const stateVec_t& X, const commandVec_t& U)
{
    lx = Q*X;
    lu = R*U;
}

void CostFunctionPneumaticarmElbow::computeFinalCostDeriv(const stateVec_t& X)
{
    lx = Q*X;
    
}

