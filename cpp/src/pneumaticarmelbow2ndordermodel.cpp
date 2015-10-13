#include "pneumaticarm2ordermodel.h"
#include <math.h>

#define pi M_PI

Pneumaticarm2orderModel::Pneumaticarm2orderModel(double& mydt)
{
    dt = mydt;
    Id.setIdentity();
    
    I = m*link_l*link_l/3;
    A.setZero();
    A(0,1) = 1.0;
    A(1,0) = -2*K2*Pm/I;
    A(1,1) = -fv/I; 
    A10 = dt*(m*g*link_l/I);

    /*A(0,1) = 1.0;
    A(2,3) = 1.0;
    A(1,0) = -((k/Jl)+(k/(Jm*R*R)));
    A(1,1) = -(fvm/Jm);
    A(1,3) = -((fvm*k)/Jm);
    A(3,0) = 1.0/Jl;
    Ad = (A*dt+Id);

    A13atan = dt*(2.0*Jm*R/(pi*Jl))*Cf0;
    A33atan = dt*(2.0/(pi*Jl))*Cf0;
*/
    B <<  0.0,
          2*K1/I;

    Ad = (A*dt + Id);
    Bd = dt*B;

    fxBase <<  1,                       dt,
               dt*(-2*K2*Pm/I),     1 -(fv/I)*dt;
    fxx[0].setZero();
    fxx[1].setZero();
    //fxx[2].setZero();
    //fxx[3].setZero();

    fxu[0].setZero();
    fxu[0].setZero();
    fuBase << 0.0,
              2*K1/I;
    fu = dt* fuBase;
    fuu[0].setZero();
    fux[0].setZero();
    fxu[0].setZero();

    QxxCont.setZero();
    QuuCont.setZero();
    QuxCont.setZero();
}


stateVec_t Pneumaticarm2orderModel::computeNextState(double& dt, const stateVec_t& X,const commandVec_t& U)
{
    stateVec_t result = Ad*X + Bd*U;
    result(1,0)-=A10*sin(X(0));
    //result(3,0)+=A33atan*atan(a*X(3,0));

    return result;
}

void Pneumaticarm2orderModel::computeAllModelDeriv(double& dt, const stateVec_t& X,const commandVec_t& U)
{
    fx = fxBase;
    fx(1,0) -= A10*cos(X(0));
    fxx[0](1,0)+= A10*sin(X(0));
    //fxx[3](1,3) = -((2*dt*Jm*R)/(pi*Jl))*Cf0*((2*a*a*a*X(3,0))/((1+(a*a*X(3,0)*X(3,0)))*(1+(a*a*X(3,0)*X(3,0)))));
    //fxx[3](3,3) = +((2*dt*Cf0)/(pi*Jl))*((2*a*a*a*X(3,0))/((1+(a*a*X(3,0)*X(3,0)))*(1+(a*a*X(3,0)*X(3,0)))));
}

stateMat_t Pneumaticarm2orderModel::computeTensorContxx(const stateVec_t& nextVx)
{
    QxxCont = nextVx[1]*fxx[1];
    return QxxCont;
}

commandMat_t Pneumaticarm2orderModel::computeTensorContuu(const stateVec_t& nextVx)
{
    return QuuCont;
}

commandR_stateC_t Pneumaticarm2orderModel::computeTensorContux(const stateVec_t& nextVx)
{
    return QuxCont;
}

/// accessors ///
unsigned int Pneumaticarm2orderModel::getStateNb()
{
    return stateNb;
}

unsigned int Pneumaticarm2orderModel::getCommandNb()
{
    return commandNb;
}

stateMat_t& Pneumaticarm2orderModel::getfx()
{
    return fx;
}

stateTens_t& Pneumaticarm2orderModel::getfxx()
{
    return fxx;
}

stateR_commandC_t& Pneumaticarm2orderModel::getfu()
{
    return fu;
}

stateR_commandC_commandD_t& Pneumaticarm2orderModel::getfuu()
{
    return fuu;
}

stateR_stateC_commandD_t& Pneumaticarm2orderModel::getfxu()
{
    return fxu;
}

stateR_commandC_stateD_t& Pneumaticarm2orderModel::getfux()
{
    return fux;
}
