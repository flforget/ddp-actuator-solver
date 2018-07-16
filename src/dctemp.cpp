#include <math.h>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <sys/time.h>

#include <iostream>

#define pi M_PI

/*
 * x0 -> actuator position
 * x1 -> actuator speed
 * x2 -> motor temperature
 * x3 -> external torque
 * x4 -> ambiant temperature
 */

#include <ddp-actuator-solver/dctemp.h>

DCTemp::DCTemp(double& mydt,bool noiseOnParameters)
{
    stateNb=5;
    commandNb=1;
    dt = mydt;
    struct timeval tv;

    if(!noiseOnParameters)
    {
        J = 119e-7;
        K_M=77.1e-3;
        f_VL=0.429e-6;
        R_th=2.8;
        tau_th=15.7;
    }
    else
    {
        J = 119e-17;
        K_M=77.1e-3;
        f_VL=0.429e-6;
        R_th=2.8;
        tau_th=15.7;
    }

    Id.setIdentity();


    fu.setZero();
    fx.setZero();

    fxx[0].setZero();
    fxx[1].setZero();
    fxx[2].setZero();
    fxx[3].setZero();
    fxx[4].setZero();

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

DCTemp::stateVec_t DCTemp::computeDeriv(double& dt, const stateVec_t& X, const commandVec_t &U)
{
    dX[0] = X[1];
    dX[1] = (K_M/J)*U[0] - (f_VL/J)*X[1] - (1.0/J)*X[3];
    dX[2] = R_th*U[0]*U[0] - (X[2]-X[4])/tau_th;
    dX[3] = 0.0;
    dX[4] = 0.0;
    //std::cout << dX.transpose() << std::endl;
    return dX;
}

DCTemp::stateVec_t DCTemp::computeNextState(double& dt, const stateVec_t& X,const commandVec_t& U)
{
    k1 = computeDeriv(dt,X,U);
    k2 = computeDeriv(dt,X+(dt/2)*k1,U);
    k3 = computeDeriv(dt,X+(dt/2)*k2,U);
    k4 = computeDeriv(dt,X+dt*k3,U);
    x_next = X + (dt/6)*(k1+2*k2+2*k3+k4);
    return x_next;
}

void DCTemp::computeAllModelDeriv(double& dt, const stateVec_t& X,const commandVec_t& U)
{
    double dh = 1e-7;
    stateVec_t Xp,Xm;
    Xp = X;
    Xm = X;
    for(int i=0;i<stateNb;i++)
    {
        Xp[i] += dh/2;
        Xm[i] -= dh/2;
        fx.col(i) = (computeNextState(dt, Xp, U) - computeNextState(dt, Xm, U))/dh;
        Xp = X;
        Xm = X;
    }
}

DCTemp::stateMat_t DCTemp::computeTensorContxx(const stateVec_t& nextVx)
{
    return QxxCont;
}

DCTemp::commandMat_t DCTemp::computeTensorContuu(const stateVec_t& nextVx)
{
    return QuuCont;
}

DCTemp::commandR_stateC_t DCTemp::computeTensorContux(const stateVec_t& nextVx)
{
    return QuxCont;
}
