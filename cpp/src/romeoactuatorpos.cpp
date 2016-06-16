#include "romeoactuatorpos.h"
#include <math.h>

#define pi M_PI

const double RomeoActuatorPos::k=1000.0;
const double RomeoActuatorPos::R=200.0;
const double RomeoActuatorPos::Jm=138*1e-7;
const double RomeoActuatorPos::Jl=0.1;
const double RomeoActuatorPos::fvm=0.01;
const double RomeoActuatorPos::Cf0=0.1;
const double RomeoActuatorPos::a=10.0;

RomeoActuatorPos::RomeoActuatorPos(double& mydt)
{
    stateNb=4;
    commandNb=1;
    dt = mydt;
    Id.setIdentity();

    A.setZero();
    A << 0.0, 1.0,0.0,0.0,
         -k/(Jm*R*R),0.0,k/(R*Jm),0.0,
         0.0,0.0,0.0,1.0,
         k/(Jl*R),0.0,-k/Jl,0.0;
    Ad = (A*dt+Id);
    fx = Ad;


    B <<  0.0,
          1.0/Jm,
          0.0,
          0.0;
    Bd = dt*B;
    fu = Bd;

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

    lowerCommandBounds << -40.0;
    upperCommandBounds << 40.0;
}


stateVec_t RomeoActuatorPos::computeNextState(double& dt, const stateVec_t& X,const stateVec_t& Xdes,const commandVec_t& U)
{
    stateVec_t result = Ad*X + Bd*U;

    return result;
}

void RomeoActuatorPos::computeAllModelDeriv(double& dt, const stateVec_t& X,const stateVec_t& Xdes,const commandVec_t& U)
{

}

stateMat_t RomeoActuatorPos::computeTensorContxx(const stateVec_t& nextVx)
{
    return QxxCont;
}

commandMat_t RomeoActuatorPos::computeTensorContuu(const stateVec_t& nextVx)
{
    return QuuCont;
}

commandR_stateC_t RomeoActuatorPos::computeTensorContux(const stateVec_t& nextVx)
{
    return QuxCont;
}
