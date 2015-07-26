#include <iostream>

#include "config.h"

#include "ilqrsolver.h"
#include "romeosimpleactuator.h"
#include "costfunctionromeoactuator.h"
#include <Eigen/Dense>

#include <time.h>
#include <sys/time.h>

using namespace std;
using namespace Eigen;

int main()
{
    struct timeval tbegin,tend;
    double texec=0.0;
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

    gettimeofday(&tbegin,NULL);
    testSolverRomeoActuator.solveTrajectory();
    gettimeofday(&tend,NULL);

    texec=((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;

    cout << "temps d'execution total du solveeur ";
    cout << texec << endl;
    cout << "temps d'execution par pas de temps ";
    cout << texec/T << endl;

    return 0;
}

