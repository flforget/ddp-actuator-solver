#ifndef DYNAMICMODEL_H
#define DYNAMICMODEL_H

#include "config.h"

#include <Eigen/Dense>

using namespace Eigen;

class DynamicModel
{
// constructors //
public:
    DynamicModel();

// attributes //
public:
private:
    unsigned int stateNb;
    unsigned int commandNb;
    double dt;

protected:

// methods //
public:
    virtual stateVec_t computeNextState(double& dt, const stateVec_t& X,const commandVec_t& U)=0;
    virtual void computeAllModelDeriv(double& dt, const stateVec_t& X,const commandVec_t& U)=0;
    virtual stateMat_t computeTensorContxx(const stateVec_t& nextVx)=0;
    virtual commandMat_t computeTensorContuu(const stateVec_t& nextVx)=0;
    virtual commandR_stateC_t computeTensorContux(const stateVec_t& nextVx)=0;
private:
protected:
    // accessors //
public:
    virtual unsigned int getStateNb()=0;
    virtual unsigned int getCommandNb()=0;
    virtual stateMat_t &getfx()=0;
    virtual stateTens_t& getfxx()=0;
    virtual stateR_commandC_t &getfu()=0;
    virtual stateR_commandC_commandD_t& getfuu()=0;
    virtual stateR_stateC_commandD_t& getfxu()=0;
    virtual stateR_commandC_stateD_t& getfux()=0;
};

#endif // DYNAMICMODEL_H
