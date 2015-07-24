#include "costfunctionromeoactuator.h"

CostFunctionRomeoActuator::CostFunctionRomeoActuator()
{
    this->Q << 100.0,0.0,0.0,0.0,
                0.0,0.0,0.0,0.0,
                0.0,0.0,0.0,0.0,
                0.0,0.0,0.0,0.0;
    this->R << 0.1;

    this->lxx = this->Q;
    this->luu = this->R;
    this->lux << 0.0,0.0,0.0,0.0;
    this->lxu << 0.0,0.0,0.0,0.0;
}

void CostFunctionRomeoActuator::computeAllCostDeriv(stateVec_t& X, stateVec_t& Xdes, commandVec_t& U)
{
    this->lx = this->Q*(X-Xdes);
    this->lu = this->R*U;
}

void CostFunctionRomeoActuator::commuteFinalCostDeriv(stateVec_t& X, stateVec_t& Xdes)
{
    this->lx = this->Q*(X-Xdes);
}

// accessors //
stateVec_t CostFunctionRomeoActuator::getlx()
{
    return this->lx;
}

stateMat_t CostFunctionRomeoActuator::getlxx()
{
    return this->lxx;
}

commandVec_t CostFunctionRomeoActuator::getlu()
{
    return this->lu;
}

commandMat_t CostFunctionRomeoActuator::getluu()
{
    return this->luu;
}

stateR_commandC_t CostFunctionRomeoActuator::getlxu()
{
    return this->lxu;
}

commandR_stateC_t CostFunctionRomeoActuator::getlux()
{
    return this->lux;
}
