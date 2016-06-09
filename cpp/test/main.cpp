#include <iostream>
#include <fstream>

#include "config.h"

#include "ilqrsolver.h"
#include "pneumaticarm_2linkmodel.hh"
#include "costfunctionpneumaticarmelbow.h"

#include <time.h>
#include <sys/time.h>


using namespace std;
using namespace Eigen;

int main()
{
    struct timeval tbegin,tend;
    double texec=0.0;
    stateVec_t xinit,xDes;

    xinit << 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
    xDes << 0.5,1.0,0.0,0.0,0.0,0.0,0.0,0.0;

    unsigned int T = 50;
    double dt=5e-3;
    unsigned int iterMax = 100;
    double stopCrit = 1e-5;
    stateVecTab_t xList;
    commandVecTab_t uList;
    ILQRSolver::traj lastTraj;

    PneumaticarmNonlinearModel model(dt);
    CostFunctionPneumaticarmElbow cost;


    ILQRSolver solver(model,cost,DISABLE_FULLDDP,DISABLE_QPBOX);
    solver.FirstInitSolver(xinit,xDes,T,dt,iterMax,stopCrit);


    int N = 10;
    gettimeofday(&tbegin,NULL);
    for(int i=0;i<N;i++) solver.solveTrajectory();
    gettimeofday(&tend,NULL);

    lastTraj = solver.getLastSolvedTrajectory();
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
