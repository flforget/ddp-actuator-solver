#ifndef ROMEOSIMPLEACTUATOR_H
#define ROMEOSIMPLEACTUATOR_H

#include "config.h"

#include "dynamicmodel.h"
#include <Eigen/Dense>

using namespace Eigen;

class RomeoSimpleActuator : public DynamicModel
{
public:
    RomeoSimpleActuator();
private:
protected:

    // attributes //
public:
private:
    unsigned int stateNb;
    unsigned int commandNb;
    double k;
    double R;
    double Jm;
    double Jl;
    double fvm;
    double Cf0;
    double a;
    stateMat_t Id;
    stateMat_t A;
    stateR_commandC_t B;
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
    stateVec_t computeNextState(double dt,stateVec_t& X,commandVec_t &U);
    void computeAllModelDeriv(double dt,stateVec_t& X,commandVec_t &U);
    // accessors //
    unsigned int getStateNb();
    unsigned int getCommandNb();
    stateMat_t &getfx();
    stateTens_t* getfxx();
    stateR_commandC_t &getfu();
    stateR_commandC_commandD_t* getfuu();
    stateR_stateC_commandD_t* getfxu();
    stateR_commandC_stateD_t* getfux();
private:
protected:

};

#endif // ROMEOSIMPLEACTUATOR_H
