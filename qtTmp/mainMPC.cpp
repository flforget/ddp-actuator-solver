#include <iostream>
#include <fstream>

#include "config.h"

#include "ilqrsolver.h"
#include "romeosimpleactuator.h"
#include "romeolinearactuator.h"
#include "costfunctionromeoactuator.h"
#include <Eigen/Dense>

#include <time.h>
#include <sys/time.h>


using namespace std;
using namespace Eigen;

int mainMPC()
{
    struct timeval tbegin,tend;
    double texec=0.0;
    stateVec_t xinit,xDes;

    xinit << 0.0,0.0,0.0,0.0;
    xDes << 1.0,0.0,0.0,0.0;

    unsigned int T = 5;
    unsigned int M = 30;
    double dt=1e-4;
    unsigned int iterMax = 20;
    double stopCrit = 1e-3;
    stateVec_t* xList;
    commandVec_t* uList;
    ILQRSolver::traj lastTraj;

    RomeoSimpleActuator romeoActuatorModel;
    RomeoLinearActuator romeoLinearModel;
    CostFunctionRomeoActuator costRomeoActuator;
    ILQRSolver testSolverRomeoActuator(romeoActuatorModel,costRomeoActuator);



    ofstream fichier("results.csv",ios::out | ios::trunc);
    if(!fichier)
    {
        cerr << "erreur fichier ! " << endl;
        return 1;
    }
    fichier << "tau,tauDot,q,qDot,u" << endl;



    gettimeofday(&tbegin,NULL);
    for(int i=0;i<M;i++)
    {
        cout << i << endl;
        testSolverRomeoActuator.initSolver(xinit,xDes,T,dt,iterMax,stopCrit);
        testSolverRomeoActuator.solveTrajectory();
        lastTraj = testSolverRomeoActuator.getLastSolvedTrajectory();
        xList = lastTraj.xList;
        uList = lastTraj.uList;
        xinit = xList[1];
        for(int j=0;j<T;j++) fichier << xList[i](0,0) << "," << xList[i](1,0) << "," << xList[i](2,0) << "," << xList[i](3,0) << "," << uList[i](0,0) << endl;
        fichier << xList[T](0,0) << "," << xList[T](1,0) << "," << xList[T](2,0) << "," << xList[T](3,0) << "," << 0.0 << endl;
        fichier << endl;
    }
    gettimeofday(&tend,NULL);


    //texec=((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;
    texec = (double)(tend.tv_usec - tbegin.tv_usec);

    cout << "temps d'execution total du solveur ";
    cout << texec << endl;
    cout << "temps d'execution par pas de MPC ";
    cout << texec/T << endl;

    fichier.close();









    return 0;

}

