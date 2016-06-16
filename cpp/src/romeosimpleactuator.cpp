#include "romeosimpleactuator.h"
#include <math.h>

#define pi M_PI

const double RomeoSimpleActuator::k=1000.0;
const double RomeoSimpleActuator::R=200.0;
const double RomeoSimpleActuator::Jm=138*1e-7;
const double RomeoSimpleActuator::Jl=0.1;
const double RomeoSimpleActuator::fvm=0.01;
const double RomeoSimpleActuator::Cf0=0.1;
const double RomeoSimpleActuator::a=10.0;

RomeoSimpleActuator::RomeoSimpleActuator(double& mydt)
{
    stateNb=4;
    commandNb=1;
    dt = mydt;
    Id.setIdentity();

    A.setZero();
    A(0,1) = 1.0;
    A(2,3) = 1.0;
    A(1,0) = -((k/Jl)+(k/(Jm*R*R)));
    A(1,1) = -(fvm/Jm);
    A(1,3) = -((fvm*k)/Jm);
    A(3,0) = 1.0/Jl;
    Ad = (A*dt+Id);

    A13atan = dt*(2.0*Jm*R/(pi*Jl))*Cf0;
    A33atan = dt*(2.0/(pi*Jl))*Cf0;

    B <<  0.0,
          k/(R*Jm),
          0.0,
          0.0;
    Bd = dt*B;

    fxBase <<   1.0,      dt,      0.0,      0.0,
                dt*(-(k/Jl)-(k/(Jm*R*R))),     1 - dt*(fvm/Jm),      0.0,      -dt*((fvm*k)/Jm),
                0.0,      0.0,      1.0,      dt,
                dt/Jl,      0.0,      0.0,      1.0;

    fxx[0].setZero();
    fxx[1].setZero();
    fxx[2].setZero();
    fxx[3].setZero();

    fxu[0].setZero();
    fxu[0].setZero();
    fuBase << 0.0,
              k/(R*Jm),
              0.0,
              0.0;
    fu = dt* fuBase;
    fuu[0].setZero();
    fux[0].setZero();
    fxu[0].setZero();

    QxxCont.setZero();
    QuuCont.setZero();
    QuxCont.setZero();

    lowerCommandBounds << -50.0;
    upperCommandBounds << 50.0;
}


stateVec_t RomeoSimpleActuator::computeNextState(double& dt, const stateVec_t& X,const stateVec_t& Xdes,const commandVec_t& U)
{
    stateVec_t result = Ad*X + Bd*U;
    result(1,0)+=A13atan*atan(a*(X(3,0)+Xdes(3,0)));
    result(3,0)+=A33atan*atan(a*(X(3,0)+Xdes(3,0)));
    return result;
}

void RomeoSimpleActuator::computeAllModelDeriv(double& dt, const stateVec_t& X,const stateVec_t& Xdes,const commandVec_t& U)
{
    Xreal = X + Xdes;
    fx = fxBase;
    fx(1,3) += A13atan*(a/(1+a*a*Xreal(3,0)*Xreal(3,0)));
    fx(3,3) -= A33atan*(a/(1+(a*a*Xreal(3,0)*Xreal(3,0))));
    fxx[3](1,3) = -((2*dt*Jm*R)/(pi*Jl))*Cf0*((2*a*a*a*Xreal(3,0))/((1+(a*a*Xreal(3,0)*Xreal(3,0)))*(1+(a*a*Xreal(3,0)*Xreal(3,0)))));
    fxx[3](3,3) = +((2*dt*Cf0)/(pi*Jl))*((2*a*a*a*Xreal(3,0))/((1+(a*a*Xreal(3,0)*Xreal(3,0)))*(1+(a*a*Xreal(3,0)*Xreal(3,0)))));
}

stateMat_t RomeoSimpleActuator::computeTensorContxx(const stateVec_t& nextVx)
{
    QxxCont = nextVx[3]*fxx[3];
    return QxxCont;
}

commandMat_t RomeoSimpleActuator::computeTensorContuu(const stateVec_t& nextVx)
{
    return QuuCont;
}

commandR_stateC_t RomeoSimpleActuator::computeTensorContux(const stateVec_t& nextVx)
{
    return QuxCont;
}
