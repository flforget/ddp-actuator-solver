#include "dynamicmodel.h"

/*DynamicModel::DynamicModel()
{
}*/
unsigned int DynamicModel::getStateNb()
{
    return stateNb;
}

unsigned int DynamicModel::getCommandNb()
{
    return commandNb;
}

commandVec_t& DynamicModel::getLowerCommandBounds()
{
    return lowerCommandBounds;
}

commandVec_t& DynamicModel::getUpperCommandBounds()
{
    return upperCommandBounds;
}

stateMat_t& DynamicModel::getfx()
{
    return fx;
}

stateTens_t& DynamicModel::getfxx()
{
    return fxx;
}

stateR_commandC_t& DynamicModel::getfu()
{
    return fu;
}

stateR_commandC_commandD_t& DynamicModel::getfuu()
{
    return fuu;
}

stateR_stateC_commandD_t& DynamicModel::getfxu()
{
    return fxu;
}

stateR_commandC_stateD_t& DynamicModel::getfux()
{
    return fux;
}
