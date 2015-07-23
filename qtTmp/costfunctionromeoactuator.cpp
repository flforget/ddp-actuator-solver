#include "costfunctionromeoactuator.h"

CostFunctionRomeoActuator::CostFunctionRomeoActuator()
{
    this->Q << 100.0,0.0,0.0,0.0,
                0.0,0.0,0.0,0.0,
                0.0,0.0,0.0,0.0,
                0.0,0.0,0.0,0.0;
    this->R = 0.1;

    this->lxx = this->Q;
    this->luu = this->R;
    this->lux << 0.0,0.0,0.0,0.0;
    this->lxu << 0.0,0.0,0.0,0.0;
}

void CostFunctionRomeoActuator::computeAllCostDeriv(V4 &X, V4 &Xdes, double &U)
{
    this->lx = this->Q*(X-Xdes);
    this->lu = this->R*U;
}

void CostFunctionRomeoActuator::commuteFinalCostDeriv(V4 &X, V4 &Xdes)
{
    this->lx = this->Q*(X-Xdes);
}

// accessors //
V4 CostFunctionRomeoActuator::getlx()
{
    return this->lx;
}

M4 CostFunctionRomeoActuator::getlxx()
{
    return this->lxx;
}

double CostFunctionRomeoActuator::getlu()
{
    return this->lu;
}

double CostFunctionRomeoActuator::getluu()
{
    return this->luu;
}

V4 CostFunctionRomeoActuator::getlxu()
{
    return this->lxu;
}

V4T CostFunctionRomeoActuator::getlux()
{
    return this->lux;
}
