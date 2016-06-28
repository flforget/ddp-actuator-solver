// Copydight (c) 2015 LAAS_CNRS //
// Author: Ganesh Kumar //


/* This is a program to model the pneumatic muscle based joint of the Pneumatic 7R arm */
#ifndef PNEUMATICARMNONLINEARMODEL_H
#define PNEUMATICARMNONLINEARMODEL_H
#include <iostream>
#include "config.h"
#include <vector>
#include "dynamicmodel.h"
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;
class PneumaticarmNonlinearModel : public DynamicModel
{
public:
    PneumaticarmNonlinearModel(double& mydt);
private:
protected:

    // attributes //
public:

private:
    double dt;
   // Muscle parameters
    double lo, alphao, k,ro,R,a,b,emax,lb,lt,epsb,epst;
    double time_constant1, time_constant2;

   
   
    std::vector<double> x1;

    stateVec_t Xreal;
    stateMat_t QxxCont;
    commandMat_t QuuCont;
    commandR_stateC_t QuxCont;

protected:
    // methods //
public:
    stateVec_t computeNextState(double& dt, const stateVec_t& X, const stateVec_t& Xdes, const commandVec_t &U);
    void computeAllModelDeriv(double& dt, const stateVec_t& X, const stateVec_t& Xdes, const commandVec_t &U);
    stateMat_t computeTensorContxx(const stateVec_t& nextVx);
    commandMat_t computeTensorContuu(const stateVec_t& nextVx);
    commandR_stateC_t computeTensorContux(const stateVec_t& nextVx);
    double getX(unsigned int i);
private:
protected:
        // accessors //
public:

};

#endif // PNEUMATICARMNONLINEARMODEL_H

