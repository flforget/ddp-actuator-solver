#include "romeolinearactuator.h"
#include <math.h>

#define pi M_PI

const double RomeoLinearActuator::k=1000.0;
const double RomeoLinearActuator::R=200.0;
const double RomeoLinearActuator::Jm=138*1e-7;
const double RomeoLinearActuator::Jl=0.1;
const double RomeoLinearActuator::Cf0=0.1;
const double RomeoLinearActuator::a=10.0;

RomeoLinearActuator::RomeoLinearActuator(double& mydt)
{
    stateNb=4;
    commandNb=1;
    dt = mydt;
    Id.setIdentity();

    A.setZero();
    A <<    0.0, 1.0, 0.0, 0.0;
            -(k/Jl)-(k/(Jm*R*R)), 0.0, 0.0, 0.0;
            0.0, 0.0, 0.0, 1.0;
            1.0/Jl, 0.0, 0.0, 0.0;
    fx = (A*dt+Id);

    B <<  0.0,
            k/(R*Jm),
            0.0,
            0.0;
    fu = dt*B;


    fxx[0].setZero();
    fxx[1].setZero();
    fxx[2].setZero();
    fxx[3].setZero();

    fuu[0].setZero();
    fux[0].setZero();
    fxu[0].setZero();

    lowerCommandBounds << -50.0;
    upperCommandBounds << 50.0;
}

stateVec_t RomeoLinearActuator::computeNextState(double& dt, const stateVec_t& X,const stateVec_t& Xdes,const commandVec_t& U)
{
    stateVec_t result = fx*X + fu*U;
    return result;
}

void RomeoLinearActuator::computeAllModelDeriv(double& dt, const stateVec_t& X,const stateVec_t& Xdes,const commandVec_t& U)
{

}

stateMat_t RomeoLinearActuator::computeTensorContxx(const stateVec_t& nextVx)
{
    stateMat_t QxxCont;
    QxxCont.setZero();
    return QxxCont;
}

commandMat_t RomeoLinearActuator::computeTensorContuu(const stateVec_t& nextVx)
{
    commandMat_t QuuCont;
    QuuCont.setZero();
    return QuuCont;
}

commandR_stateC_t RomeoLinearActuator::computeTensorContux(const stateVec_t& nextVx)
{
    commandR_stateC_t QuxCont;
    QuxCont.setZero();
    return QuxCont;
}
