#ifndef COSTFUNCTIONPNEUMATICARMELBOW_H
#define COSTFUNCTIONPNEUMATICARMELBOW_H

#include "config.h"

#include "costfunction.h"

#include <Eigen/Dense>

using namespace Eigen;

class CostFunctionPneumaticarmElbow : public CostFunction
{
public:
    CostFunctionPneumaticarmElbow();
private:
    stateMat_t Q;
    stateMat_t Qf;
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

#endif
