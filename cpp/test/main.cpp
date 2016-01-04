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
    struct timeval tbegin,tend;
    double texec=0.0;
    stateVec_t xinit,xDes;

    xinit << -1.0,0.0,0.0,0.0;
    xDes << 2.0,0.0,0.0,0.0;

    unsigned int T = 300;
    double dt=1e-4;
    unsigned int iterMax = 20;
    double stopCrit = 1e-3;
    stateVec_t* xList;
    commandVec_t* uList;
    ILQRSolver::traj lastTraj;

    RomeoSimpleActuator romeoActuatorModel(dt);
    RomeoLinearActuator romeoLinearModel(dt);
    CostFunctionRomeoActuator costRomeoActuator;
    ILQRSolver testSolverRomeoActuator(romeoActuatorModel,costRomeoActuator);
    testSolverRomeoActuator.FirstInitSolver(xinit,xDes,T,dt,iterMax,stopCrit);


    int N = 100;
    gettimeofday(&tbegin,NULL);
    for(int i=0;i<N;i++) testSolverRomeoActuator.solveTrajectory();
    gettimeofday(&tend,NULL);

    lastTraj = testSolverRomeoActuator.getLastSolvedTrajectory();
    xList = lastTraj.xList;
    uList = lastTraj.uList;
    unsigned int iter = lastTraj.iter;

    texec=((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;
    texec /= N;

    cout << endl;
    cout << "temps d'execution total du solveur ";
    cout << texec << endl;
    cout << "temps d'execution par pas de temps ";
    cout << texec/T << endl;
    cout << "Nombre d'itérations : " << iter << endl;





    ofstream fichier("results.csv",ios::out | ios::trunc);
    if(fichier)
    {
        fichier << "tau,tauDot,q,qDot,u" << endl;
        for(int i=0;i<T;i++) fichier << xList[i](0,0) << "," << xList[i](1,0) << "," << xList[i](2,0) << "," << xList[i](3,0) << "," << uList[i](0,0) << endl;
        fichier << xList[T](0,0) << "," << xList[T](1,0) << "," << xList[T](2,0) << "," << xList[T](3,0) << "," << 0.0 << endl;
        fichier.close();
    }
    else
        cerr << "erreur ouverte fichier" << endl;
    return 0;

}
