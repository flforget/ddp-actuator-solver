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
    //static const unsigned int stateNb=4;
    //static const unsigned int commandNb=1;
public:
    static const double k;
    static const double R;
    static const double Jm;
    static const double Jl;
    double fvm;
    static const double Cf0;
    static const double a;
private:
    stateMat_t Id;
    stateMat_t A;
    stateR_commandC_t B;

protected:
    // methods //
public:
    stateVec_t computeNextState(double& dt,const stateVec_t& X,const stateVec_t& Xdes,const commandVec_t &U);
    void computeAllModelDeriv(double& dt, const stateVec_t& X,const stateVec_t& Xdes,const commandVec_t &U);
    stateMat_t computeTensorContxx(const stateVec_t& nextVx);
    commandMat_t computeTensorContuu(const stateVec_t& nextVx);
    commandR_stateC_t computeTensorContux(const stateVec_t& nextVx);
private:
protected:
        // accessors //
public:

};

#endif // ROMEOLINEARACTUATOR_H
