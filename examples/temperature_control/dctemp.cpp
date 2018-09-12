#include <math.h>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <sys/time.h>

#include <iostream>

#include "dctemp.hh"

/*
 * x0 -> actuator position
 * x1 -> actuator speed
 * x2 -> motor temperature
 * x3 -> external torque
 * x4 -> ambiant temperature
 */

DCTemp::DCTemp(bool noiseOnParameters)
{
  stateNb = 5;
  commandNb = 1;
  if (!noiseOnParameters)
  {
    J_ = 119e-7;
    K_M_ = 77.1e-3;
    f_VL_ = 0.429e-6;
    R_th_ = 2.8;
    tau_th_ = 15.7;
  }
  else
  {
    J_ = 119e-17;
    K_M_ = 77.1e-3;
    f_VL_ = 0.429e-6;
    R_th_ = 2.8;
    tau_th_ = 15.7;
  }

  Id_.setIdentity();

  fu.setZero();
  fx.setZero();

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

  QxxCont_.setZero();
  QuuCont_.setZero();
  QuxCont_.setZero();

  lowerCommandBounds << -1.0;
  upperCommandBounds << 1.0;
}

DCTemp::stateVec_t DCTemp::computeDeriv(double& ,
                                        const stateVec_t& X,
                                        const commandVec_t &U)
{
  dX_[0] = X[1];
  dX_[1] = (K_M_ / J_) * U[0] - (f_VL_ / J_) * X[1] - (1.0 / J_) * X[3];
  dX_[2] = R_th_ * U[0] * U[0] - (X[2] - X[4]) / tau_th_;
  dX_[3] = 0.0;
  dX_[4] = 0.0;
  //std::cout << dX.transpose() << std::endl;
  return dX_;
}

DCTemp::stateVec_t DCTemp::computeNextState(double& dt,
    const stateVec_t& X,
    const commandVec_t& U)
{
  k1_ = computeDeriv(dt, X, U);
  k2_ = computeDeriv(dt, X + (dt / 2) * k1_, U);
  k3_ = computeDeriv(dt, X + (dt / 2) * k2_, U);
  k4_ = computeDeriv(dt, X + dt * k3_, U);
  x_next_ = X + (dt_ / 6) * (k1_ + 2 * k2_ + 2 * k3_ + k4_);
  return x_next_;
}

void DCTemp::computeAllModelDeriv(double& dt,
                                  const stateVec_t& X,
                                  const commandVec_t& U)
{
  double dh = 1e-7;
  stateVec_t Xp, Xm;
  Xp = X;
  Xm = X;
  for (unsigned int i = 0; i < stateNb; i++)
  {
    Xp[i] += dh / 2;
    Xm[i] -= dh / 2;
    fx.col(i) = (computeNextState(dt, Xp, U) - computeNextState(dt, Xm, U)) / dh;
    Xp = X;
    Xm = X;
  }
}

DCTemp::stateMat_t DCTemp::computeTensorContxx(const stateVec_t& )
{
  return QxxCont_;
}

DCTemp::commandMat_t DCTemp::computeTensorContuu(const stateVec_t& )
{
  return QuuCont_;
}

DCTemp::commandR_stateC_t DCTemp::computeTensorContux(const stateVec_t& )
{
  return QuxCont_;
}
