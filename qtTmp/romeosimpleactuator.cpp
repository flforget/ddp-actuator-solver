#include "romeosimpleactuator.h"
#include <math.h>

#define pi 3,141593

RomeoSimpleActuator::RomeoSimpleActuator()
{
    this->commandNb = 1;
    this->stateNb = 4;
    this->k = 1000.0;
    this->R = 200.0;
    this->Jm = 138*1e-7;
    this->Jl = 0.1;
    this->fvm = 0.01;
    this->Cf0 = 0.1;
    this->a = 10.0;

    this->Id <<     1.0,    0.0,      0.0,      0.0,
                    0.0,      1.0,    0.0,      0.0,
                    0.0,      0.0,      1.0,    0.0,
                    0.0,      0.0,      0.0,      1.0;

    this->A <<  0.0,      1.0,      0.0,      0.0,
                0.0,      0.0,      0.0,      0.0,
                0.0,      0.0,      1.0,      0.0,
                0.0,      0.0,      0.0,      0.0;
    this->A(1,0) = -(this->k/this->Jl)+(this->k/(this->Jm*this->R*this->R));
    this->A(1,3) = -this->fvm*this->k/this->Jm;
    this->A(3,0) =1.0/this->Jl;
    this->A13atan = 2.0*this->Jm*this->R/(pi*this->Jl)*this->Cf0;
    this->A33atan = 2.0/(pi*this->Jl)*this->Cf0;

    this->B <<  0.0,
                this->k/(this->R*this->Jm),
                0.0,
                0.0;

    this->fxBase << 1.0,      1.0,      0.0,      0.0,
                    -(this->k/this->Jl)-(this->k/(this->Jm*this->R*this->R)),      -this->fvm/this->Jm,      0.0,      -this->fvm*this->k/this->Jm,
                    0.0,      0.0,      1.0,      1.0,
                    1.0/this->Jl,      0.0,      0.0,      0.0;
    this->fx = this->fxBase;
    this->fxx[0] << 0.0,      0.0,      0.0,      0.0,
                    0.0,      0.0,      0.0,      0.0,
                    0.0,      0.0,      0.0,      0.0,
                    0.0,      0.0,      0.0,      0.0;
    this->fxx[1] = this->fxx[0];
    this->fxx[2] = this->fxx[0];
    this->fxx[3] = this->fxx[0];
    this->fxu[0] <<    0.0,      0.0,      0.0,      0.0,
                    0.0,      0.0,      0.0,      0.0,
                    0.0,      0.0,      0.0,      0.0,
                    0.0,      0.0,      0.0,      0.0;
    this->fxu[0] <<    0.0,      0.0,      0.0,      0.0,
                    0.0,      0.0,      0.0,      0.0,
                    0.0,      0.0,      0.0,      0.0,
                    0.0,      0.0,      0.0,      0.0;
    this->fuBase << 0.0,
                    this->k/(this->R*this->Jm),
                    0.0,
                    0.0;
    this->fu << 0.0,0.0,0.0,0.0;


}


stateVec_t RomeoSimpleActuator::computeNextState(double dt,stateVec_t& X,commandVec_t& U)
{
    stateMat_t Ad = (this->A*dt+this->Id);
    stateR_commandC_t Bd = dt*this->B;
    stateVec_t result = Ad*X + Bd*U;
    result(1,0)+=this->A13atan*atan(this->a*X(3,0));
    result(3,0)+=this->A33atan*atan(this->a*X(3,0));

    return result;
}

void RomeoSimpleActuator::computeAllModelDeriv(double dt,stateVec_t& X,commandVec_t& U)
{
    this->fx = this->fxBase;

    this->fx(0,1) = dt;
    this->fx(1,0) *= dt;
    this->fx(1,1) *= dt;
    this->fx(1,1) += 1.0;
    this->fx(1,3) *= dt;
    this->fx(1,3) += ((2*dt*this->Jm*this->R)/(pi*this->Jl))*this->Cf0*(this->a/(1.0+(this->a*this->a*X(3,0)*X(3,0))));
    this->fx(2,3) *= dt;
    this->fx(3,0) *= dt;
    this->fx(3,3) += 1.0-((2*dt*this->Cf0)/(pi*this->Jl))*(this->a/(1.0+(this->a*this->a*X(3,0)*X(3,0))));

    this->fu = dt*this->fuBase;

    this->fxx[3](3,3) = -((2*dt*this->Jm*this->R)/(pi*this->Jl))*this->Cf0*((2*this->a*this->a*this->a*X(3,0))/((1+(this->a*this->a*X(3,0)*X(3,0)))*(1+(this->a*this->a*X(3,0)*X(3,0)))));
    this->fxx[3](3,3) = +((2*dt*this->Cf0)/(pi*this->Jl))*((2*this->a*this->a*this->a*X(3,0))/((1+(this->a*this->a*X(3,0)*X(3,0)))*(1+(this->a*this->a*X(3,0)*X(3,0)))));
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

stateMat_t RomeoSimpleActuator::getfx()
{
    return this->fx;
}

stateTens_t* RomeoSimpleActuator::getfxx()
{
    return &this->fxx;
}

stateR_commandC_t RomeoSimpleActuator::getfu()
{
    return this->fu;
}

stateR_commandC_commandD_t* RomeoSimpleActuator::getfuu()
{
    return &this->fuu;
}

stateR_stateC_commandD_t* RomeoSimpleActuator::getfxu()
{
    return &this->fxu;
}

stateR_commandC_stateD_t* RomeoSimpleActuator::getfux()
{
    return &this->fux;
}
