#ifndef COSTFUNCTIONROMEOACTUATOR_H
#define COSTFUNCTIONROMEOACTUATOR_H

#include "costfunction.h"

class CostFunctionRomeoActuator : public CostFunction<double,4,1>
{
public:
    CostFunctionRomeoActuator();
private:
    stateMat_t Q;
    commandMat_t R;
    double dt;
protected:
    // attributes //
public:
private:

protected:
    // methods //
public:
    void computeAllCostDeriv(const stateVec_t& X,const stateVec_t& Xdes, const commandVec_t& U);
    void computeFinalCostDeriv(const stateVec_t& X,const stateVec_t& Xdes);
private:
protected:
    // accessors //
public:

};

#endif // COSTFUNCTIONROMEOACTUATOR_H
