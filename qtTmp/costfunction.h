#ifndef COSTFUNCTION_H
#define COSTFUNCTION_H

#include "config.h"

class CostFunction
{
public:
    CostFunction();
private:
protected:
    // attributes //
public:
private:
protected:
    // methods //
public:
    virtual void computeAllCostDeriv(stateVec_t& X, stateVec_t& Xdes, commandVec_t& U)=0;
    virtual void commuteFinalCostDeriv(stateVec_t& X, stateVec_t& Xdes)=0;
private:
protected:
    // accessors //
public:
    virtual stateVec_t getlx()=0;
    virtual stateMat_t getlxx()=0;
    virtual commandVec_t getlu()=0;
    virtual commandMat_t getluu()=0;
    virtual commandR_stateC_t getlux()=0;
    virtual stateR_commandC_t getlxu()=0;
};

#endif // COSTFUNCTION_H
