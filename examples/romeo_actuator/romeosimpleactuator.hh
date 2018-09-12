#ifndef ROMEOSIMPLEACTUATOR_H
#define ROMEOSIMPLEACTUATOR_H

#include <ddp-actuator-solver/dynamicmodel.hh>

class RomeoSimpleActuator: public DynamicModel<double, 4, 1>
{
public:
  RomeoSimpleActuator(double& mydt, bool noiseOnParameters = 0);
  virtual ~RomeoSimpleActuator() {};
private:
protected:

  // attributes //
public:
private:
  double dt;
private:
  double k;
  double R;
  double Jm;
  double Jl;
  double fvm;
  double fvl;
  double Kt;
  double mu;
  double Cf0;
  double a;
private:
  stateVec_t Xreal;
  stateMat_t Id;
  stateMat_t A;
  stateMat_t Ad;
  stateR_commandC_t B;
  stateR_commandC_t Bd;
  double A13atan;
  double A33atan;
  stateMat_t fxBase;
  stateR_commandC_t fuBase;

  stateMat_t QxxCont;
  commandMat_t QuuCont;
  commandR_stateC_t QuxCont;

protected:
  // methods //
public:
  stateVec_t computeNextState(double& dt,
                              const stateVec_t& X,
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

#endif // ROMEOSIMPLEACTUATOR_H
