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
    virtual void computeAllCostDeriv(stateVec_t& X, stateVec_t& Xdes, commandVec_t& U);
    virtual void commuteFinalCostDeriv(stateVec_t& X, stateVec_t& Xdes);
private:
protected:
    // accessors //
public:
    virtual stateVec_t getlx();
    virtual stateMat_t getlxx();
    virtual commandVec_t getlu();
    virtual commandMat_t getluu();
    virtual commandR_stateC_t getlux();
    virtual stateR_commandC_t getlxu();
};

#endif // COSTFUNCTION_H
