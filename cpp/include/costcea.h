#ifndef COSTCEA_H
#define COSTCEA_H

#include "costfunction.h"

class CostCea : public CostFunction<double,4,2>
{
public:
    CostCea();
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
    void computeAllCostDeriv(const stateVec_t& X, const commandVec_t& U);
    void computeFinalCostDeriv(const stateVec_t& X);
private:
protected:
    // accessors //
public:

};

#endif // COSTFUNCTIONROMEOACTUATOR_H
