#include "pneumaticarm2nonlinearmodel.h"
#include <math.h>

#define pi M_PI

Pneumaticarm2NonlinearModel::Pneumaticarm2NonlinearModel(double& mydt)
{
    stateNb=2;
    commandNb=2;
    
   //////////////////////////////////////////////
   // double lo, alphao, k,ro,R,a,b,emax,lb,lt,epsb,epst;
    lo = 0.185;
    alphao = 20.0*pi/180;
    k = 1.25;
    ro = 0.0085;
    R = 0.015;
    a = 3/pow((tan(alphao)),2);
    b = 1/pow((sin(alphao)),2);
    emax = (1/k)*(1 - sqrt(b/a));
   
  
//Parameters of Joint
    m = 2.6;
    link_l = 0.32;
    g  = 9.81;
    I = m*pow(link_l,2)/3;
    fv = 0.25;
   //////////////////////////////////////////////
    time_constant1 = 0.18;
    time_constant2 = 0.13;
    dt = mydt;
    Id.setIdentity();

    A.setZero();
    A(0,1) = 1.0;
    A(1,1) = -fv/I; 

    fxx[0].setZero();
    fxx[1].setZero();
    //fxx[2].setZero();
    //fxx[3].setZero();

    fxu[0].setZero();
    fxu[0].setZero();

    fuu[0].setZero();
    fux[0].setZero();
    fxu[0].setZero();

    QxxCont.setZero();
    QuuCont.setZero();
    QuxCont.setZero();
}


stateVec_t Pneumaticarm2NonlinearModel::computeNextState(double& dt, const stateVec_t& X,const commandVec_t& U)
{
    stateVec_t result;
    stateVec_t jointstate_deriv;
    double co,theta, theta_dot, tb1,tb2,tb3,tt1,tt2_1,tt2,tt3_1,tt3,F1,F2,T,P1,P2,u1,u2, Tc1, Tc2;    
    theta = X(0);
    theta_dot = X(1);
    P1 = U(0);
    P2 = U(1);
   
    Tc1 = time_constant1;
    Tc2 = time_constant2;
    lb = lo - R*theta;
    epsb = (1-(lb/lo));
    lt = lo*(1-emax) + R*theta;
    epst = (1-(lt/lo));

    co = pi*ro*ro; 
    double t1_j21, t1, t2;
    t1_j21  = - m*g*0.5*link_l*cos(theta)/I;
    t1 = -co*P1*(2*a*k*(1-k*epsb)*(R/lo));
    t2 = -co*P2*(2*a*k*(1-k*epst)*(R/lo));
    A(1,0) = t1_j21 + (t1 + t2)*(R/I);
    B(0,0) = 0.0;
    B(0,1) = 0.0;
    B(1,0) =  (R/I)*pi*ro*ro*(a*pow((1-k*epsb),2) - b);
    B(1,1) =  (-R/I)*pi*ro*ro*(a*pow((1-k*epst),2) - b);
    Ad = (A*dt + Id);
    Bd = B*dt;
    //stateVec_t result = Ad*X + Bd*U;*/
    F1 =  pi*ro*ro*P1*(a*pow((1-k*epsb),2) - b);
    F2 =  pi*ro*ro*P2*(a*pow((1-k*epst),2) - b);
    jointstate_deriv(0) = theta_dot; //%joint_state(2);
    jointstate_deriv(1) = ((F1 -F2 )*R  - fv*theta_dot - m*g*0.5*link_l*sin(theta))/I;
    
    //result = X + dt*jointstate_deriv;
    result = Ad*X + Bd*U;
    fx = Ad;
    fu = Bd;
    return result;
}

void Pneumaticarm2NonlinearModel::computeAllModelDeriv(double& dt, const stateVec_t& X,const commandVec_t& U)
{
    double co,theta,P1,P2;    
    theta = X(0);
    P1 = U(0);
    P2 = U(1);
    co = pi*ro*ro;
    fxx[0](1,0) = (m*g*0.5*link_l*sin(theta)/I) + 2*co*a*pow((k*R/lo),2)*(P1 - P2)*(R/I);
   
}

stateMat_t Pneumaticarm2NonlinearModel::computeTensorContxx(const stateVec_t& nextVx)
{
    QxxCont = nextVx[0]*fxx[0];// + nextVx[1]*fxx[1];// + nextVx[2]*fxx[2] + nextVx[3]*fxx[3];
    return QxxCont;
}

commandMat_t Pneumaticarm2NonlinearModel::computeTensorContuu(const stateVec_t& nextVx)
{
    return QuuCont;
}

commandR_stateC_t Pneumaticarm2NonlinearModel::computeTensorContux(const stateVec_t& nextVx)
{
    return QuxCont;
}

/// accessors ///
unsigned int Pneumaticarm2NonlinearModel::getStateNb()
{
    return stateNb;
}

unsigned int Pneumaticarm2NonlinearModel::getCommandNb()
{
    return commandNb;
}

stateMat_t& Pneumaticarm2NonlinearModel::getfx()
{
    return fx;
}

stateTens_t& Pneumaticarm2NonlinearModel::getfxx()
{
    return fxx;
}

stateR_commandC_t& Pneumaticarm2NonlinearModel::getfu()
{
    return fu;
}

stateR_commandC_commandD_t& Pneumaticarm2NonlinearModel::getfuu()
{
    return fuu;
}

stateR_stateC_commandD_t& Pneumaticarm2NonlinearModel::getfxu()
{
    return fxu;
}

stateR_commandC_stateD_t& Pneumaticarm2NonlinearModel::getfux()
{
    return fux;
}
