#include "romeosimpleactuator.h"
#include <math.h>

#define pi M_PI

/*const double RomeoSimpleActuator::k=588.0;
const double RomeoSimpleActuator::R=91.2;
const double RomeoSimpleActuator::Jm=138*1e-7;
const double RomeoSimpleActuator::Jl=0.00008;
const double RomeoSimpleActuator::fvm=0.003;
const double RomeoSimpleActuator::Cf0=0.0;
const double RomeoSimpleActuator::a=0.0;*/


const double RomeoSimpleActuator::k=588.0;
const double RomeoSimpleActuator::R=96.1;
const double RomeoSimpleActuator::Jm=183*1e-7;
const double RomeoSimpleActuator::Jl=0.000085;
const double RomeoSimpleActuator::fvm=5.65e-5;
const double RomeoSimpleActuator::fvl=0.278;
const double RomeoSimpleActuator::Kt=0.0578;
const double RomeoSimpleActuator::mu=0.52;
const double RomeoSimpleActuator::Cf0=0.0;
const double RomeoSimpleActuator::a=0.0;

/*
 * x0 -> actuator position
 * x1 -> actuator speed
 * x2 -> motor position
 * x3 -> motor speed
 */

RomeoSimpleActuator::RomeoSimpleActuator(double& mydt)
{
    stateNb=4;
    commandNb=1;
    dt = mydt;
    Id.setIdentity();

    A <<   0.0,1.0,0.0,0.0,
            -k/Jl,-fvl/Jl,k/(R*Jl),0.0,
            0.0,0.0,0.0,1.0,
            k/(R*Jm),0.0,-k/(Jm*R*R),-fvm/Jm;
    B << 0.0,0.0,0.0,Kt/Jm;


    fu << 0.0,0.0,0.0,Kt/Jm;
    fx.setZero();

    fxx[0].setZero();
    fxx[1].setZero();
    fxx[2].setZero();
    fxx[3].setZero();

    fxu[0].setZero();
    fxu[0].setZero();

    fuu[0].setZero();
    fux[0].setZero();
    fxu[0].setZero();

    QxxCont.setZero();
    QuuCont.setZero();
    QuxCont.setZero();

    lowerCommandBounds << -1.0;
    upperCommandBounds << 1.0;
}


RomeoSimpleActuator::stateVec_t RomeoSimpleActuator::computeNextState(double& dt, const stateVec_t& X,const stateVec_t& Xdes,const commandVec_t& U)
{
    Xreal = X + Xdes;
    stateVec_t x_next,k1,k2,k3,k4;
    k1 = A*Xreal + B*U;
    k2 = A*(Xreal+(dt/2)*k1) + B*U;
    k3 = A*(Xreal+(dt/2)*k2) + B*U;
    k4 = A*(Xreal+dt*k3) + B*U;
    x_next = Xreal + (dt/6)*(k1+2*k2+2*k3+k4) - Xdes;

    return x_next;
}

void RomeoSimpleActuator::computeAllModelDeriv(double& dt, const stateVec_t& X,const stateVec_t& Xdes,const commandVec_t& U)
{
    double dh = 1e-7;
    stateVec_t Xp,Xm;
    Xp = X;
    Xm = X;
    for(int i=0;i<4;i++)
    {
        Xp[i] += dh/2;
        Xm[i] -= dh/2;
        fx.col(i) = (computeNextState(dt, Xp, Xdes, U) - computeNextState(dt, Xm, Xdes, U))/dh;
        Xp = X;
        Xm = X;
    }
}

RomeoSimpleActuator::stateMat_t RomeoSimpleActuator::computeTensorContxx(const stateVec_t& nextVx)
{
    return QxxCont;
}

RomeoSimpleActuator::commandMat_t RomeoSimpleActuator::computeTensorContuu(const stateVec_t& nextVx)
{
    return QuuCont;
}

RomeoSimpleActuator::commandR_stateC_t RomeoSimpleActuator::computeTensorContux(const stateVec_t& nextVx)
{
    return QuxCont;
}
