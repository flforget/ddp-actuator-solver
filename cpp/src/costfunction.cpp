#include "costfunction.h"

CostFunction::CostFunction()
{
}

stateVec_t& CostFunction::getlx()
{
    return lx;
}

stateMat_t& CostFunction::getlxx()
{
    return lxx;
}

commandVec_t& CostFunction::getlu()
{
    return lu;
}

commandMat_t& CostFunction::getluu()
{
    return luu;
}

commandR_stateC_t& CostFunction::getlux()
{
    return lux;
}

stateR_commandC_t& CostFunction::getlxu()
{
    return lxu;
}
