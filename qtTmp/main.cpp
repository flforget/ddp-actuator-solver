#include <iostream>

#include "config.h"

#include "ilqrsolver.h"
#include "romeosimpleactuator.h"
#include "costfunctionromeoactuator.h"
#include <Eigen/Dense>

#include <time.h>

using namespace std;
using namespace Eigen;

int main()
{
    clock_t t1,t2;
    t1 = clock();
    stateVec_t xinit,xDes;

    xinit << 0.0,0.0,0.0,0.0;
    xDes << 1.0,0.0,0.0,0.0;

    unsigned int T = 50;
    double dt=1e-4;
    int iterMax = 20;
    double stopCrit = 1e-3;

    RomeoSimpleActuator romeoActuatorModel;
    CostFunctionRomeoActuator costRomeoActuator;
    ILQRSolver testSolverRomeoActuator(romeoActuatorModel,costRomeoActuator);
    testSolverRomeoActuator.initSolver(xinit,xDes,T,dt,iterMax,stopCrit);
    testSolverRomeoActuator.solveTrajectory();

    usleep(1000000);
    t2 = clock();

    cout << t2 << endl;
    cout << t1 << endl;

    return 0;
}

