#include "pneumaticarmelbowlinear.h"
#include <math.h>

#define pi M_PI

PneumaticarmElbowLinear::PneumaticarmElbowLinear(double& mydt)
{
    dt = mydt;
    Id.setIdentity();

    A.setZero();
    A(0,0) = 0; //-10.0133221748582;
    A(0,1) = 1; //-10.2586175950466;
    A(0,2) = 0; //-5.46841588650530;
    A(1,0) = 0; //8;
    A(1,2) = 1;
    A(2,0) = -1001;
    A(2,1) = -148.2;
    A(2,2) = -18.69;
    //A(2,1) = 8;
    Ad = (A*dt+Id);

    B <<    0.0,
            0.0 ,
            522.50;
    Bd = dt*B;
    fx = (A*dt+Id);
    fxx[0].setZero();
    fxx[1].setZero();
    fxx[2].setZero();
   /* fxBase <<  (1-(10.0133221748582*dt)),	-10.2586175950466*dt,	-5.46841588650530*dt,
                                8*dt,	                1,	                0,
                                0,	                8*dt,	                1;*/
   fxBase <<  1,	     dt,     	    0,
              0,	    1,              dt,
            -1001*dt,       -148.2*dt,    (1- dt*18.69);
    fuBase <<   0.0,
                0.0,
                522.5;
    fu = dt*fuBase;
    fuu[0].setZero();
    fux[0].setZero();
    fxu[0].setZero();
}

stateVec_t PneumaticarmElbowLinear::computeNextState(double& dt, const stateVec_t& X,const commandVec_t& U)
{
    stateVec_t result = Ad*X + Bd*U;
    return result;
}

void PneumaticarmElbowLinear::computeAllModelDeriv(double& dt, const stateVec_t& X,const commandVec_t& U)
{

}

stateMat_t PneumaticarmElbowLinear::computeTensorContxx(const stateVec_t& nextVx)
{
    stateMat_t QxxCont;
    QxxCont.setZero();
    return QxxCont;
}

commandMat_t PneumaticarmElbowLinear::computeTensorContuu(const stateVec_t& nextVx)
{
    commandMat_t QuuCont;
    QuuCont.setZero();
    return QuuCont;
}

commandR_stateC_t PneumaticarmElbowLinear::computeTensorContux(const stateVec_t& nextVx)
{
    commandR_stateC_t QuxCont;
    QuxCont.setZero();
    return QuxCont;
}

/// accessors ///
unsigned int PneumaticarmElbowLinear::getStateNb()
{
    return stateNb;
}

unsigned int PneumaticarmElbowLinear::getCommandNb()
{
    return commandNb;
}

stateMat_t& PneumaticarmElbowLinear::getfx()
{
    return fx;
}

stateTens_t& PneumaticarmElbowLinear::getfxx()
{
    return fxx;
}

stateR_commandC_t& PneumaticarmElbowLinear::getfu()
{
    return fu;
}

stateR_commandC_commandD_t& PneumaticarmElbowLinear::getfuu()
{
    return fuu;
}

stateR_stateC_commandD_t& PneumaticarmElbowLinear::getfxu()
{
    return fxu;
}

stateR_commandC_stateD_t& PneumaticarmElbowLinear::getfux()
{
    return fux;
}
