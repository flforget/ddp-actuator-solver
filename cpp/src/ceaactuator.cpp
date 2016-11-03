#include "ceaactuator.h"
#include <math.h>

#define pi M_PI

const double CeaActuator::k=588.0;
const double CeaActuator::R=91.2;
const double CeaActuator::Jm=138*1e-7;
const double CeaActuator::Jl=0.0323;
const double CeaActuator::fvm=0.003;
const double CeaActuator::fvl=0.0037;

CeaActuator::CeaActuator(double& mydt)
{
    stateNb=4;
    commandNb=2;
    dt = mydt;
    Id.setIdentity();

    A.setZero();
    A <<    0.0,0.0,1.0,0.0,
            0.0,-(fvm/Jm)-(fvl/(Jm*R*R)),   fvl/(R*Jm),     -k/(Jm*R*R),
            0.0,fvl/(Jl*R),                 -fvl/Jl,        k/(R*Jl),
            0.0,1.0,                        -R,             0.0;
    Ad = (A*dt+Id);
    fx = Ad;


    B <<    0.0,0.0,
            1/Jm,0.0,
            0.0,1/Jl,
            0.0,0.0;
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


CeaActuator::stateVec_t CeaActuator::computeNextState(double& dt, const stateVec_t& X,const stateVec_t& Xdes,const commandVec_t& U)
{
    stateVec_t result = Ad*X + Bd*U;

    return result;
}

void CeaActuator::computeAllModelDeriv(double& dt, const stateVec_t& X,const stateVec_t& Xdes,const commandVec_t& U)
{

}

CeaActuator::stateMat_t CeaActuator::computeTensorContxx(const stateVec_t& nextVx)
{
    return QxxCont;
}

CeaActuator::commandMat_t CeaActuator::computeTensorContuu(const stateVec_t& nextVx)
{
    return QuuCont;
}

CeaActuator::commandR_stateC_t CeaActuator::computeTensorContux(const stateVec_t& nextVx)
{
    return QuxCont;
}
