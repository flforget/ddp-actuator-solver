#ifndef DYNAMICMODEL_H
#define DYNAMICMODEL_H

#include "config.h"

#include <Eigen/Dense>

using namespace Eigen;

class DynamicModel
{
// constructors //
public:
    //DynamicModel();

// attributes //
public:
protected:
    unsigned int stateNb;
    unsigned int commandNb;
    double dt;
    commandVec_t lowerCommandBounds;
    commandVec_t upperCommandBounds;

    stateMat_t fx;
    stateTens_t fxx;
    stateR_commandC_t fu;
    stateR_commandC_commandD_t fuu;
    stateR_stateC_commandD_t fxu;
    stateR_commandC_stateD_t fux;
public:


protected:


// methods //
public:
    virtual stateVec_t computeNextState(double& dt, const stateVec_t& X,const stateVec_t& Xdes,const commandVec_t& U)=0;
    virtual void computeAllModelDeriv(double& dt, const stateVec_t& X,const stateVec_t& Xdes,const commandVec_t& U)=0;
    virtual stateMat_t computeTensorContxx(const stateVec_t& nextVx)=0;
    virtual commandMat_t computeTensorContuu(const stateVec_t& nextVx)=0;
    virtual commandR_stateC_t computeTensorContux(const stateVec_t& nextVx)=0;
private:
protected:
    // accessors //
public:
    unsigned int getStateNb();
    unsigned int getCommandNb();
    commandVec_t& getLowerCommandBounds();
    commandVec_t& getUpperCommandBounds();
    stateMat_t& getfx();
    stateTens_t& getfxx();
    stateR_commandC_t &getfu();
    stateR_commandC_commandD_t& getfuu();
    stateR_stateC_commandD_t& getfxu();
    stateR_commandC_stateD_t& getfux();
};

#endif // DYNAMICMODEL_H
