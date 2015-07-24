#ifndef DYNAMICMODEL_H
#define DYNAMICMODEL_H

#include "config.h"

#include <Eigen/Core>

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
    virtual stateVec_t computeNextState(unsigned int dt,stateVec_t& X,commandVec_t& U);
    virtual void computeAllModelDeriv(unsigned int dt,stateVec_t& X,commandVec_t& U);
    virtual unsigned int getStateNb();
    virtual unsigned int getCommandNb();
private:
protected:
};

#endif // DYNAMICMODEL_H
