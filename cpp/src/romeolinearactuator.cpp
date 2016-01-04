#include "romeolinearactuator.h"
#include <math.h>

#define pi M_PI

RomeoLinearActuator::RomeoLinearActuator(double& mydt)
{
    dt = mydt;
    Id.setIdentity();

    A.setZero();
    A(0,1) = 1.0;
    A(2,2) = 1.0;
    A(1,0) = -(k/Jl)+(k/(Jm*R*R));
    A(3,0) =1.0/Jl;
    Ad = (A*dt+Id);

    B <<  0.0,
                k/(R*Jm),
                0.0,
                0.0;
    Bd = dt*B;

    fxBase << 1.0,      1.0,      0.0,      0.0,
                    -(k/Jl)-(k/(Jm*R*R)),      0.0,      0.0,      0.0,
                    0.0,      0.0,      1.0,      1.0,
                    1.0/Jl,      0.0,      0.0,      0.0;
    fx = (A*dt+Id);
    fxx[0].setZero();
    fxx[1].setZero();
    fxx[2].setZero();
    fxx[3].setZero();


    fuBase <<   0.0,
                k/(R*Jm),
                0.0,
                0.0;
    fu = dt*fuBase;
    fuu[0].setZero();
    fux[0].setZero();
    fxu[0].setZero();
}

stateVec_t RomeoLinearActuator::computeNextState(double& dt, const stateVec_t& X,const stateVec_t& Xdes,const commandVec_t& U)
{
    stateVec_t result = Ad*(X+Xdes) + Bd*U;
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

/// accessors ///
unsigned int RomeoLinearActuator::getStateNb()
{
    return stateNb;
}

unsigned int RomeoLinearActuator::getCommandNb()
{
    return commandNb;
}

stateMat_t& RomeoLinearActuator::getfx()
{
    return fx;
}

stateTens_t& RomeoLinearActuator::getfxx()
{
    return fxx;
}

stateR_commandC_t& RomeoLinearActuator::getfu()
{
    return fu;
}

stateR_commandC_commandD_t& RomeoLinearActuator::getfuu()
{
    return fuu;
}

stateR_stateC_commandD_t& RomeoLinearActuator::getfxu()
{
    return fxu;
}

stateR_commandC_stateD_t& RomeoLinearActuator::getfux()
{
    return fux;
}
