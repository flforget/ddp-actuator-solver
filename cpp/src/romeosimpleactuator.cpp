#include "romeosimpleactuator.h"
#include <math.h>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <sys/time.h>

#define pi M_PI

/*
 * x0 -> actuator position
 * x1 -> actuator speed
 * x2 -> motor position
 * x3 -> motor speed
 */

RomeoSimpleActuator::RomeoSimpleActuator(double& mydt,bool noiseOnParameters)
{
    stateNb=4;
    commandNb=1;
    dt = mydt;
    struct timeval tv;

    if(!noiseOnParameters)
    {
        k = 588.0;
        R = 96.1;
        Jm = 183 * 1e-7;
        Jl = 0.000085;
        fvm = 5.65e-5;
        fvl = 0.278;
        Kt = 0.0578;
        mu = 0.52;
        Cf0 = 0.0;
        a = 0.0;
    }
    else
    {
        gettimeofday(&tv,NULL);
        srand(tv.tv_usec);
        k = 588.0 + 0.0*588.0*0.1*(2.0*(rand()/(double)RAND_MAX)-1.0);
        R = 96.1 + 96.1*0.1*(2.0*(rand()/(double)RAND_MAX)-1.0);
        Jm = 183*1e-7 + 183*1e-7*0.1*(2.0*(rand()/(double)RAND_MAX)-1.0);
        Jl = 0.000085 + 0.0*0.000085*0.1*(2.0*(rand()/(double)RAND_MAX)-1.0);
        fvm = 5.65e-5 + 5.65e-5*0.1*(2.0*(rand()/(double)RAND_MAX)-1.0);
        fvl = 0.278 + 0.0*0.278*0.1*(2.0*(rand()/(double)RAND_MAX)-1.0);
        Kt = 0.0578 + 0.0578*0.1*(2.0*(rand()/(double)RAND_MAX)-1.0);
        mu = 0.52;
        Cf0 = 0.0;
        a = 0.0;
    }

    Id.setIdentity();

    A <<   0.0,1.0,0.0,0.0,
            -k/Jl,-fvl/Jl,k/(R*Jl),0.0,
            0.0,0.0,0.0,1.0,
            k/(R*Jm),0.0,-k/(Jm*R*R),-fvm/Jm;

    Ad = (dt*A).exp();

    B << 0.0,0.0,0.0,Kt/Jm;
    Bd = dt*B;

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


RomeoSimpleActuator::stateVec_t RomeoSimpleActuator::computeNextState(double& dt, const stateVec_t& X,const commandVec_t& U)
{
    stateVec_t x_next;//,k1,k2,k3,k4;
    /*k1 = A*X + B*U;
    k2 = A*(X+(dt/2)*k1) + B*U;
    k3 = A*(X+(dt/2)*k2) + B*U;
    k4 = A*(X+dt*k3) + B*U;
    x_next = X + (dt/6)*(k1+2*k2+2*k3+k4);*/

    x_next = Ad*X + Bd*U;
    return x_next;
}

void RomeoSimpleActuator::computeAllModelDeriv(double& dt, const stateVec_t& X,const commandVec_t& U)
{
    double dh = 1e-7;
    stateVec_t Xp,Xm;
    Xp = X;
    Xm = X;
    for(int i=0;i<4;i++)
    {
        Xp[i] += dh/2;
        Xm[i] -= dh/2;
        fx.col(i) = (computeNextState(dt, Xp, U) - computeNextState(dt, Xm, U))/dh;
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
