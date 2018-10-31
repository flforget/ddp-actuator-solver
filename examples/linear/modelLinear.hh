#ifndef MODEL_H
#define MODEL_H

#include <ddp-actuator-solver/dynamicmodel.hh>

class ModelLinear : public DynamicModel<double, 2, 1>
{
public:
  ModelLinear(double& mydt, bool noiseOnParameters = 0);
  virtual ~ModelLinear() {};
private:
protected:

  // attributes //
public:
private:
  double dt;
  double l;
  double M;
  double m;
  double g;
private:
  stateVec_t Xreal, dX;
  stateVec_t x_next, k1, k2, k3, k4;
  stateMat_t Id;

  stateMat_t QxxCont;
  commandMat_t QuuCont;
  commandR_stateC_t QuxCont;

protected:
  // methods //
public:
  stateVec_t computeDeriv(double& dt, const stateVec_t& X, const commandVec_t &U);
  stateVec_t computeNextState(double& dt, const stateVec_t& X,
                              const commandVec_t &U);
  void computeModelDeriv(double& dt, const stateVec_t& X, const commandVec_t &U);
  stateMat_t computeTensorContxx(const stateVec_t& nextVx);
  commandMat_t computeTensorContuu(const stateVec_t& nextVx);
  commandR_stateC_t computeTensorContux(const stateVec_t& nextVx);
private:
protected:
  // accessors //
public:

};

#endif // MODEL_H
