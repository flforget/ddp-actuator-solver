#ifndef ROMEOLINEARACTUATOR_H
#define ROMEOLINEARACTUATOR_H

#include "config.h"

#include "dynamicmodel.h"
#include <Eigen/Dense>

using namespace Eigen;

class RomeoLinearActuator : public DynamicModel
{
public:
    RomeoLinearActuator(double& mydt);
private:
protected:

    // attributes //
public:
private:
    double dt;
    static const unsigned int stateNb=4;
    static const unsigned int commandNb=1;
    static const double k=1000.0;
    static const double R=200.0;
    static const double Jm=138*1e-7;
    static const double Jl=0.1;
    double fvm;
    static const double Cf0=0.1;
    static const double a=10.0;

    stateMat_t Id;
    stateMat_t A;
    stateMat_t Ad;
    stateR_commandC_t B;
    stateR_commandC_t Bd;
    double A13atan;
    double A33atan;
    stateMat_t fx,fxBase;
    stateTens_t fxx;
    stateR_commandC_t fu,fuBase;
    stateR_commandC_commandD_t fuu;
    stateR_stateC_commandD_t fxu;
    stateR_commandC_stateD_t fux;

protected:
    // methods //
public:
    stateVec_t computeNextState(double& dt,const stateVec_t& X,const commandVec_t &U);
    void computeAllModelDeriv(double& dt, const stateVec_t& X,const commandVec_t &U);
    stateMat_t computeTensorContxx(const stateVec_t& nextVx);
    commandMat_t computeTensorContuu(const stateVec_t& nextVx);
    commandR_stateC_t computeTensorContux(const stateVec_t& nextVx);
private:
protected:
        // accessors //
public:
    unsigned int getStateNb();
    unsigned int getCommandNb();
    stateMat_t &getfx();
    stateTens_t& getfxx();
    stateR_commandC_t &getfu();
    stateR_commandC_commandD_t& getfuu();
    stateR_stateC_commandD_t& getfxu();
    stateR_commandC_stateD_t& getfux();

};

#endif // ROMEOLINEARACTUATOR_H
