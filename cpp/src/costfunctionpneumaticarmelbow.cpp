#include "costfunctionpneumaticarmelbow.h"

CostFunctionPneumaticarmElbow::CostFunctionPneumaticarmElbow()

{
     Q << 1e4*1.0, 0.0, 0.0, 0.0,
     0.0,  0.0,    0.0,    0.0,
     2.0e-1*0.0,  0.0,    1e-15*0.0,    0.0,
     5e-1*0.0,  0.0,    0.0,    1e-15*0.0;
   /* Qf << 1e5*1.0, 0.0, 0.0, 0.0,
      0.0,  0.0,    0.0,    0.0,
      0.0,  0.0,   1e-5*1.0,    0.0,
      0.0,  0.0,    0.0,    1e-5*1.0;*/

   /* Q << 1e-2*1.0, 0.0,
        0.0, 0.0;*/   
    Qf = Q;

    R << 1e-1, 0.0,
      0.0,  1e-1*1;

    lxx = Q;
    luu = R;
    //lux << 0.0,0.0,0.0;
    //lxu << 0.0,0.0,0.0;
    lx.setZero();
}

void CostFunctionPneumaticarmElbow::computeAllCostDeriv(const stateVec_t& X,  const commandVec_t& U)
{
   
    
    /*Q(2,2) = exp(pow(X(2),2));
    Q(3,3) = exp(pow(X(3),2));
    
    Qlx(2,2) = 2*X(2)*exp(pow(X(2),2));
    Qlx(3,3) = 2*X(3)*exp(pow(X(3),2));
    
    Qlxx(2,2) = 2*exp(pow(X(2),2))*(1 + 2*X(2)*X(2));
    Qlxx(3,3) = 2*exp(pow(X(3),2))*(1 + 2*X(3)*X(3));
    
    lx(0) = Q(0,0)*(X(0)-Xdes(0));
    lx(1) = Q(1,1)*(X(1)-Xdes(1)); 
    lx(2) = Q(2,2)*(X(2)-Xdes(2)) + 0.5*(X(2) -Xdes(2))*Qlx(2,2)*(X(2)-Xdes(2));
    lx(3) = Q(3,3)*(X(3)-Xdes(3)) + 0.5*(X(3) -Xdes(3))*Qlx(3,3)*(X(3)-Xdes(3));

    lxx(2,2) = Q(2,2) + Qlx(2,2)*(X(2)-Xdes(2)) +  0.5*(X(2) -Xdes(2))*Qlxx(2,2)*(X(2)-Xdes(2));
    lxx(3,3) = Q(3,3) + Qlx(3,3)*(X(3)-Xdes(3)) +  0.5*(X(3) -Xdes(3))*Qlxx(3,3)*(X(3)-Xdes(3));*/
    lx = Q*(X);
    lu = R*U;
}

void CostFunctionPneumaticarmElbow::computeFinalCostDeriv(const stateVec_t& X)
{
   
   /* Qf(2,2) = exp(pow(X(2),2));
    Qf(3,3) = exp(pow(X(3),2));
    
    Qflx(2,2) = 2*X(2)*exp(pow(X(2),2));
    Qflx(3,3) = 2*X(3)*exp(pow(X(3),2));
    
    Qflxx(2,2) = 2*exp(pow(X(2),2))*(1 + 2*X(2)*X(2));
    Qflxx(3,3) = 2*exp(pow(X(3),2))*(1 + 2*X(3)*X(3));
    
    lx(0) = Qf(0,0)*(X(0)-Xdes(0));
    lx(1) = Qf(1,1)*(X(1)-Xdes(1)); 
    lx(2) = Qf(2,2)*(X(2)-Xdes(2)) + 0.5*(X(2) -Xdes(2))*Qflx(2,2)*(X(2)-Xdes(2));
    lx(3) = Qf(3,3)*(X(3)-Xdes(3)) + 0.5*(X(3) -Xdes(3))*Qflx(3,3)*(X(3)-Xdes(3));*/
    lx = Qf*(X);
       
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
