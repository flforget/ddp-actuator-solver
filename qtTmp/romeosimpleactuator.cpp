#include "romeosimpleactuator.h"

RomeoSimpleActuator::RomeoSimpleActuator()
{
    this->commandNb = 1;
    this->stateNb = 4;
    this->k = 1000.0;
    this->R = 200.0;
    this->Jm = 138*1e-7;
    this->Jl = 0.1;
    this->Cf0 = 0.1;
    this->a = 10.0;
}


void RomeoSimpleActuator::computeNextState(unsigned int dt,Eigen::VectorXd X,unsigned int U)
{

}

void RomeoSimpleActuator::computeAllModelDeriv(unsigned int dt,unsigned int X,unsigned int U)
{

}

/// accessors ///
unsigned int RomeoSimpleActuator::getStateNb()
{
    return this->stateNb;
}

unsigned int RomeoSimpleActuator::getCommandNb()
{
    return this->commandNb;
}
