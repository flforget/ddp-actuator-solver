#include "pneumaticarmelbowpiecelinear.h"
#include <math.h>

#define pi M_PI

PneumaticarmElbowPieceLinear::PneumaticarmElbowPieceLinear(double& mydt)
{
    dt = mydt;
    Id.setIdentity();

    A.setZero();
    A(0,0) = -10.0133221748582;
    A(0,1) = -10.2586175950466;
    A(0,2) = -5.46841588650530;
    A(1,0) = 8;
    A(2,1) = 8;
 
   // Simwith initpres1 
   A1 <<  -11.0012135167941,   -10.6116333989229,   -6.23857734425803,   
                         8,                   0,                   0,   
                         0,                   8,                   0;   
    
    Ad1= (A1*dt+Id);

    B1 <<   0.5,
            0.0,
            0.0;
    Bd1 = dt*B;
    
    
   A2 << -10.0133221748582,   -10.2586175950466,   -5.46841588650530,
                         8,                   0,                    0,
                         0,                   8,                    0;
    
    
    
    Ad2= (A2*dt+Id);

    B2 <<   0.8,
            0.0,
            0.0;
    Bd2 = dt*B2;
    
    
    
    fx = (A*dt+Id);
    fxx[0].setZero();
    fxx[1].setZero();
    fxx[2].setZero();
    fxBase <<  (1-(10.0133221748582*dt)),	-10.2586175950466*dt,	-5.46841588650530*dt,
                                8*dt,	                1,	                0,
                                0,	                8*dt,	                1;

    fuBase <<   2.0,
                0.0,
                0.0;
    fu = dt*fuBase;
    fuu[0].setZero();
    fux[0].setZero();
    fxu[0].setZero();
}

stateVec_t PneumaticarmElbowPieceLinear::computeNextState(double& dt, unsigned int operatingpoint, const stateVec_t& X,const commandVec_t& U)
{
   unsigned int i = operatingpoint;
    stateVec_t result = Ad(i)*X + Bd(i)*U;
    return result;
}

void PneumaticarmElbowPieceLinear::computeAllModelDeriv(double& dt, const stateVec_t& X,const commandVec_t& U)
{

}

stateMat_t PneumaticarmElbowPieceLinear::computeTensorContxx(const stateVec_t& nextVx)
{
    stateMat_t QxxCont;
    QxxCont.setZero();
    return QxxCont;
}

commandMat_t PneumaticarmElbowPieceLinear::computeTensorContuu(const stateVec_t& nextVx)
{
    commandMat_t QuuCont;
    QuuCont.setZero();
    return QuuCont;
}

commandR_stateC_t PneumaticarmElbowPieceLinear::computeTensorContux(const stateVec_t& nextVx)
{
    commandR_stateC_t QuxCont;
    QuxCont.setZero();
    return QuxCont;
}

/// accessors ///
unsigned int PneumaticarmElbowPieceLinear::getStateNb()
{
    return stateNb;
}

unsigned int PneumaticarmElbowPieceLinear::getCommandNb()
{
    return commandNb;
}

stateMat_t& PneumaticarmElbowPieceLinear::getfx()
{
    return fx;
}

stateTens_t& PneumaticarmElbowPieceLinear::getfxx()
{
    return fxx;
}

stateR_commandC_t& PneumaticarmElbowPieceLinear::getfu()
{
    return fu;
}

stateR_commandC_commandD_t& PneumaticarmElbowPieceLinear::getfuu()
{
    return fuu;
}

stateR_stateC_commandD_t& PneumaticarmElbowPieceLinear::getfxu()
{
    return fxu;
}

stateR_commandC_stateD_t& PneumaticarmElbowPieceLinear::getfux()
{
    return fux;
}
