#include <math.h>

#include "romeotorqueactuator.hh"
#define pi M_PI

const double RomeoTorqueActuator::k=588.0;
const double RomeoTorqueActuator::R=96.1;
const double RomeoTorqueActuator::Jm=183*1e-7;
const double RomeoTorqueActuator::Jl=0.000085;
const double RomeoTorqueActuator::fvm=5.65e-5;
const double RomeoTorqueActuator::fvl=0.278;
const double RomeoTorqueActuator::Kt=0.0578;
const double RomeoTorqueActuator::mu=0.52;
const double RomeoTorqueActuator::Cf0=0.0;
const double RomeoTorqueActuator::a=0.0;

/*
 * x0 -> spring torque
 * x1 -> actuator position
 * x2 -> actuator speed
 * x3 -> motor speed
 */

RomeoTorqueActuator::RomeoTorqueActuator(double& mydt)
{
  stateNb=4;
  commandNb=1;
  dt = mydt;

  Id.setIdentity();

  A <<    0.0,0.0,k,-k/R,
      0.0,0.0,1.0,0.0,
      -1.0/Jl,0.0,-fvl/Jl,0.0,
      1/(R*Jm),0.0,0.0,-fvm/Jm;
  B << 0.0,0.0,0.0,Kt/Jm;


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


RomeoTorqueActuator::stateVec_t RomeoTorqueActuator::computeNextState(double& dt, const stateVec_t& X,const commandVec_t& U)
{
  stateVec_t x_next,k1,k2,k3,k4;
  k1 = A*X + B*U;
  k2 = A*(X+(dt/2)*k1) + B*U;
  k3 = A*(X+(dt/2)*k2) + B*U;
  k4 = A*(X+dt*k3) + B*U;
  x_next = X + (dt/6)*(k1+2*k2+2*k3+k4);

  return x_next;
}

void RomeoTorqueActuator::computeModelDeriv(double& dt, const stateVec_t& X,const commandVec_t& U)
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

RomeoTorqueActuator::stateMat_t RomeoTorqueActuator::computeTensorContxx(const stateVec_t& )
{
  return QxxCont;
}

RomeoTorqueActuator::commandMat_t RomeoTorqueActuator::computeTensorContuu(const stateVec_t& )
{
  return QuuCont;
}

RomeoTorqueActuator::commandR_stateC_t RomeoTorqueActuator::computeTensorContux(const stateVec_t& )
{
  return QuxCont;
}
