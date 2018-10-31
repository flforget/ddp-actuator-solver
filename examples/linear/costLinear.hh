#ifndef COST_H
#define COST_H

#include <ddp-actuator-solver/costfunction.hh>

class CostLinear : public CostFunction<double,2,1>
{
public:
    CostLinear();
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
    void computeCostAndDeriv(const stateVec_t& X,const stateVec_t& Xdes, const commandVec_t& U);
    void computeFinalCostAndDeriv(const stateVec_t& X,const stateVec_t& Xdes);
private:
protected:
    // accessors //
public:

};

#endif // COST_H
