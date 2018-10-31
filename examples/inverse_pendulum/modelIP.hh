#ifndef MODEL_H
#define MODEL_H

#include <ddp-actuator-solver/dynamicmodel.hh>

class ModelIP : public DynamicModel<double,5,1>
{
  public:
    ModelIP(double& mydt,bool noiseOnParameters=0);
    virtual ~ModelIP() {};
  private:
  protected:

    // attributes //
  public:
  private:
    double dt;
  private:
    double J;
    double K_M;
    double f_VL;
    double R_th;
    double tau_th;
  private:
    stateVec_t Xreal,dX;
    stateVec_t x_next,k1,k2,k3,k4;
    stateMat_t Id;

    stateMat_t QxxCont;
    commandMat_t QuuCont;
    commandR_stateC_t QuxCont;

  protected:
    // methods //
  public:
    stateVec_t computeDeriv(double& dt, const stateVec_t& X, const commandVec_t &U);
    stateVec_t computeNextState(double& dt, const stateVec_t& X, const commandVec_t &U);
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
