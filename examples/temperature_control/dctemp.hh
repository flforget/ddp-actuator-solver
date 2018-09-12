#ifndef DCTEMP_H
#define DCTEMP_H

#include <ddp-actuator-solver/dynamicmodel.hh>

class DCTemp : public DynamicModel<double, 5, 1>
{
public:
  DCTemp(bool noiseOnParameters = 0);
  virtual ~DCTemp() {};
private:
protected:

  // attributes //
public:
private:
  double dt_;
private:
  double J_;
  double K_M_;
  double f_VL_;
  double R_th_;
  double tau_th_;
private:
  stateVec_t Xreal_, dX_;
  stateVec_t x_next_, k1_, k2_, k3_, k4_;
  stateMat_t Id_;

  stateMat_t QxxCont_;
  commandMat_t QuuCont_;
  commandR_stateC_t QuxCont_;

protected:
  // methods //
public:
  stateVec_t computeDeriv(double& dt, const stateVec_t& X, const commandVec_t &U);
  stateVec_t computeNextState(double& dt, const stateVec_t& X,
                              const commandVec_t &U);
  void computeAllModelDeriv(double& dt, const stateVec_t& X,
                            const commandVec_t &U);
  stateMat_t computeTensorContxx(const stateVec_t& nextVx);
  commandMat_t computeTensorContuu(const stateVec_t& nextVx);
  commandR_stateC_t computeTensorContux(const stateVec_t& nextVx);
private:
protected:
  // accessors //
public:

};

#endif // DCTEMP_H
