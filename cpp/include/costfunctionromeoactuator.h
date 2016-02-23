#ifndef COSTFUNCTIONROMEOACTUATOR_H
#define COSTFUNCTIONROMEOACTUATOR_H

#include "types.h"

#include "costfunction.h"

#include <Eigen/Dense>

using namespace Eigen;

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
    void computeAllCostDeriv(const stateVec_t& X, const commandVec_t& U);
    void computeFinalCostDeriv(const stateVec_t& X);
private:
protected:
    // accessors //
public:

};

#endif // COSTFUNCTIONROMEOACTUATOR_H
