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

protected:

// methods //
public:
    virtual stateVec_t computeNextState(double dt,stateVec_t& X,commandVec_t& U)=0;
    virtual void computeAllModelDeriv(double dt,stateVec_t& X,commandVec_t& U)=0;
    virtual unsigned int getStateNb()=0;
    virtual unsigned int getCommandNb()=0;
    virtual stateMat_t &getfx()=0;
    virtual stateTens_t* getfxx()=0;
    virtual stateR_commandC_t &getfu()=0;
    virtual stateR_commandC_commandD_t* getfuu()=0;
    virtual stateR_stateC_commandD_t* getfxu()=0;
    virtual stateR_commandC_stateD_t* getfux()=0;
private:
protected:
};

#endif // DYNAMICMODEL_H
