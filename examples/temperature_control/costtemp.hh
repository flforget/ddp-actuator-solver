#ifndef COSTTEMP_H
#define COSTTEMP_H

#include <ddp-actuator-solver/costfunction.hh>

class CostTemp : public CostFunction<double,5,1>
{
public:
    CostTemp();
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

#endif // COSTFUNCTIONROMEOACTUATOR_H
