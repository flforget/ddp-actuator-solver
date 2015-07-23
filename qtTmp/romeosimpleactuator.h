#ifndef ROMEOSIMPLEACTUATOR_H
#define ROMEOSIMPLEACTUATOR_H

#include <iostream>

#include "dynamicmodel.h"
#include <Eigen/Core>

using namespace Eigen;

typedef Matrix<double,4,4> M4;
typedef Matrix<double,4,1> V4;

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
    double fvm;
    double Cf0;
    double a;
    M4 Id;
    M4 A;
    V4 B;
    double A13atan;
    double A33atan;
    M4 fx,fxBase;
    M4 fxx[4];
    V4 fu,fuBase;
    V4 fuu;
    M4 fxu,fux;

protected:
    // methods //
public:
    V4 computeNextState(double dt,V4& X,double& U);
    void computeAllModelDeriv(double dt,V4& X,double& U);
    // accessors //
    unsigned int getStateNb();
    unsigned int getCommandNb();
    M4 getfx();
    M4* getfxx();
    V4 getfu();
    V4 getfuu();
    M4 getfxu();
    M4 getfux();
private:
protected:

};

#endif // ROMEOSIMPLEACTUATOR_H
