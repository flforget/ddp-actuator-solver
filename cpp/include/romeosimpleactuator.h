#ifndef ROMEOSIMPLEACTUATOR_H
#define ROMEOSIMPLEACTUATOR_H

#include "dynamicmodel.h"

class RomeoSimpleActuator : public DynamicModel<double,4,1>
{
public:
    RomeoSimpleActuator(double& mydt);
private:
protected:

    // attributes //
public:
private:
    double dt;
public:
    /*static const double k;
    static const double R;
    static const double Jm;
    static const double Jl;
    static const double fvm;
    static const double Cf0;
    static const double a;*/

    static const double k;
    static const double R;
    static const double Jm;
    static const double Jl;
    static const double fvm;
    static const double fvl;
    static const double Kt;
    static const double mu;
    static const double Cf0;
    static const double a;
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
    stateVec_t computeNextState(double& dt, const stateVec_t& X,const stateVec_t& Xdes, const commandVec_t &U);
    void computeAllModelDeriv(double& dt, const stateVec_t& X,const stateVec_t& Xdes, const commandVec_t &U);
    stateMat_t computeTensorContxx(const stateVec_t& nextVx);
    commandMat_t computeTensorContuu(const stateVec_t& nextVx);
    commandR_stateC_t computeTensorContux(const stateVec_t& nextVx);
private:
protected:
        // accessors //
public:

};

#endif // ROMEOSIMPLEACTUATOR_H
