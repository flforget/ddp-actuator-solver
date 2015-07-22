#ifndef ROMEOSIMPLEACTUATOR_H
#define ROMEOSIMPLEACTUATOR_H

#include "dynamicmodel.h"
#include <Eigen/Core>

class RomeoSimpleActuator : public DynamicModel
{
public:
    RomeoSimpleActuator();
private:
protected:

    // attributes //
public:
private:
    unsigned int stateNb;
    unsigned int commandNb;
    double k;
    double R;
    double Jm;
    double Jl;
    double Cf0;
    double a;
protected:
    //methods //
public:
    void computeNextState(unsigned int dt,Eigen::VectorXd X,unsigned int U);
    void computeAllModelDeriv(unsigned int dt,unsigned int X,unsigned int U);
    unsigned int getStateNb();
    unsigned int getCommandNb();
private:
protected:

};

#endif // ROMEOSIMPLEACTUATOR_H
