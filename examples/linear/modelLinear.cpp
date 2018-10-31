#include <math.h>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <sys/time.h>

#include <iostream>

#include "modelLinear.hh"

/*
 * x0 -> pendulum angle
 * x1 -> pendulum angular velocity
 * x2 -> kart position
 * x3 -> kart veolicity
 */

ModelLinear::ModelLinear(double& mydt, bool noiseOnParameters)
{
  stateNb = 2;
  commandNb = 1;
  dt = mydt;

  Id.setIdentity();

  fu.setZero();
  fx.setZero();

  fu << 0.0, dt;
  fx << 1.0,dt,
        0.0,1.0;

  fxx[0].setZero();
  fxx[1].setZero();

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

ModelLinear::stateVec_t ModelLinear::computeDeriv(double&, const stateVec_t& X,
    const commandVec_t &U)
{
  dX[0] = X[1];
  dX[1] = U[0];
  return dX;
}

ModelLinear::stateVec_t ModelLinear::computeNextState(double& dt,
    const stateVec_t& X,
    const commandVec_t& U)
{
  /*k1 = computeDeriv(dt, X, U);
  k2 = computeDeriv(dt, X + (dt / 2) * k1, U);
  k3 = computeDeriv(dt, X + (dt / 2) * k2, U);
  k4 = computeDeriv(dt, X + dt * k3, U);
  x_next = X + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4);*/
  x_next = fx*X + fu*U;
  return x_next;
}

void ModelLinear::computeModelDeriv(double& dt, const stateVec_t& X,
                                    const commandVec_t& U)
{
  /*double dh = 1e-7;
  stateVec_t Xp, Xm;
  commandVec_t Up, Um;
  Xp = X;
  Xm = X;
  Up = U;
  Um = U;
  for (unsigned int i = 0; i < stateNb; i++)
  {
    Xp[i] += dh / 2;
    Xm[i] -= dh / 2;
    fx.col(i) = (computeNextState(dt, Xp, U) - computeNextState(dt, Xm, U))
                / dh;
    Xp = X;
    Xm = X;
  }
  for (unsigned int i = 0; i < commandNb; i++)
  {
    Up[i] += dh / 2;
    Um[i] -= dh / 2;
    fu.col(i) = (computeNextState(dt, X, Up) - computeNextState(dt, X, Um))
                / dh;
    Up = U;
    Um = U;
  }*/
}

ModelLinear::stateMat_t ModelLinear::computeTensorContxx(const stateVec_t& )
{
  return QxxCont;
}

ModelLinear::commandMat_t ModelLinear::computeTensorContuu(const stateVec_t& )
{
  return QuuCont;
}

ModelLinear::commandR_stateC_t ModelLinear::computeTensorContux(
  const stateVec_t& )
{
  return QuxCont;
}
