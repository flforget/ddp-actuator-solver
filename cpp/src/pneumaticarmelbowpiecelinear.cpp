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
    
     
   A2 << -10.0133221748582,   -10.2586175950466,   -5.46841588650530,
                         8,                   0,                    0,
                         0,                   8,                    0;
    
   A5 << -15.4958751590178,	-7.48083038524815,	-5.36027364991789,
                        16,	                0,	                0,
                         0,	                8,              	0;   
   A7 << -16.3080485246685,	-7.53156271508541,	-5.39781634563265,
                        16,             	0,              	0,
                         0,              	8,              	0;

   A8 << -19.8663414574989,	-8.92856543924227,	-7.13272983545439,
                        16,                 	0,              	0,
                         0,             	8,              	0;

   A9 << -18.6598045562689,	-8.15069010429652,	-6.13461158697029,
                        16,             	0,	                0,
                         0,             	8,                 	0;

   A10 << -12.1659929215942,	-5.69553712442344,	-6.70960363104254,
                         16,            	0,              	0,
                          0,            	4,              	0;

   A11 << -12.3240911652486,	-5.88669655483780,	-6.86223365414504,
                         16,                	0,              	0,
                          0,                	4,              	0;
   B  <<   2.0,
           0.0,
           0.0;
   Ad1 = (A1*dt+Id);
   Ad2 = (A2*dt+Id);
   Ad5 = (A5*dt+Id);
   Ad7 = (A7*dt+Id);  

   Ad8 = (A8*dt+Id);
   Ad9 = (A9*dt+Id);
   Ad10 = (A10*dt+Id);
   Ad11 = (A11*dt+Id);
  
   Bd = dt*B;
    


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
   stateVec_t result;
   switch(operatingpoint)
   {
       case 0 : result = Ad1*X + Bd*U;
                break;
       case 1 : result = Ad2*X + Bd*U;
                break;
       case 2 : result = Ad2*X + Bd*U;
                break;
       case 3 : result = Ad2*X + Bd*U;
                break;
       case 4 : result = Ad5*X + Bd*U;
                break; 
       case 5 : result = Ad5*X + Bd*U;
                break;
       case 6 : result = Ad7*X + Bd*U;
                break;
       case 7 : result = Ad8*X + Bd*U;
                break;        
       case 8 : result = Ad9*X + Bd*U;
                break;       
       case 9 : result = Ad10*X + Bd*U;
                break;
       case 10 :result = Ad11*X + Bd*U;
                break;  
       default :result = Ad11*X + Bd*U;
   }
             
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
