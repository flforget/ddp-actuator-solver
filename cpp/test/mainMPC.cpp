#include <iostream>
#include <fstream>

#include "config.h"

#include "ilqrsolver.h"
#include "romeosimpleactuator.h"
#include "romeolinearactuator.h"
#include "costfunctionromeoactuator.h"

#include <time.h>
#include <sys/time.h>


using namespace std;
using namespace Eigen;

int main()
{
    cout << endl;
    struct timeval tbegin,tend;
    double texec=0.0;
    stateVec_t xinit,xDes;

    xinit << 0.0,0.0,0.0,0.0;
    xDes << 1.0,0.0,0.0,0.0;

    unsigned int T = 8;
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



    ofstream fichier("resultsMPC.csv",ios::out | ios::trunc);
    if(!fichier)
    {
        cerr << "erreur fichier ! " << endl;
        return 1;
    }
    fichier << T << "," << M << endl;
    fichier << "tau,tauDot,q,qDot,u" << endl;



    gettimeofday(&tbegin,NULL);
    for(int i=0;i<M;i++)
    {
        testSolverRomeoActuator.initSolver(xinit,xDes,T,dt,iterMax,stopCrit);
        testSolverRomeoActuator.solveTrajectory();
        lastTraj = testSolverRomeoActuator.getLastSolvedTrajectory();
        xList = lastTraj.xList;
        uList = lastTraj.uList;
        xinit = xList[1];
        for(int j=0;j<T;j++) fichier << xList[j](0,0) << "," << xList[j](1,0) << "," << xList[j](2,0) << "," << xList[j](3,0) << "," << uList[j](0,0) << endl;
        fichier << xList[T](0,0) << "," << xList[T](1,0) << "," << xList[T](2,0) << "," << xList[T](3,0) << "," << 0.0 << endl;
    }
    gettimeofday(&tend,NULL);


    //texec=((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;
    texec = (double)(tend.tv_usec - tbegin.tv_usec);

    cout << "temps d'execution total du solveur ";
    cout << texec/1000000.0 << endl;
    cout << "temps d'execution par pas de MPC ";
    cout << texec/(T*1000000) << endl;

    fichier.close();









    return 0;

}

