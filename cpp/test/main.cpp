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
    double tsec = 0.0; long double tusec = 0.0;
    stateVec_t xinit,xDes;

    xinit << 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
    xDes << 0.5,1.0,0.0,0.0,0.0,0.0,0.0,0.0;

    unsigned int T = 400;
    double dt=10e-3;
    unsigned int iterMax = 100;
    double stopCrit = 1e-5;
    stateVecTab_t xList1;
    commandVecTab_t uList1;
    ILQRSolver::traj lastTraj;

    PneumaticarmNonlinearModel model(dt);
    CostFunctionPneumaticarmElbow cost;


    ILQRSolver solver(model,cost,DISABLE_FULLDDP,DISABLE_QPBOX);
    solver.FirstInitSolver(xinit,xDes,T,dt,iterMax,stopCrit);
    


    int N = 1;
    gettimeofday(&tbegin,NULL);
    for(int i=0;i<N;i++) solver.solveTrajectory();
    gettimeofday(&tend,NULL);

    lastTraj = solver.getLastSolvedTrajectory();
    xList1 = lastTraj.xList;
    uList1 = lastTraj.uList;
    unsigned int iter = lastTraj.iter;

    /*texec=((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;
    texec /= N;

    cout << endl;
    cout << "temps d'execution total du solveur ";
    cout << texec << endl;
    cout << "temps d'execution par pas de temps ";
    cout << texec/T << endl;
    cout << "Nombre d'itérations : " << iter << endl;*/
    
    tsec = (double)(tend.tv_sec - tbegin.tv_sec);
    tusec = (long double)(tend.tv_usec - tbegin.tv_usec);
    if (tusec < 0.0)tsec -=1;
    texec = 1e-6*(tsec*1e6 + abs(tusec));
    
    cout << "Time of execution of solver for the entire trrajectory ";
    cout << texec<< endl;
    cout << "Time of execution of each step of MPC ";
    cout << texec/(T) << endl;
    cout << "Number of itérations : " << iter << endl;
    
    ofstream fichier("results.csv",ios::out | ios::trunc);
    if(fichier)
    {
        fichier << "pos1,pos2,vel1,vel2,u1,u2" << endl;
        for(int i=0;i<T;i++) fichier << xList1[i](0,0) << "," << xList1[i](1,0) << "," << xList1[i](2,0) << "," << xList1[i](3,0) << "," << uList1[i](0,0) << "," << uList1[i](1,0) << endl;
        fichier << xList1[T](0,0) << "," << xList1[T](1,0) << "," << xList1[T](2,0) << "," << xList1[T](3,0) << "," << 0.0 << "," << 0.0 << endl;
        fichier.close();
    }
    else
        cerr << "erreur ouverte fichier" << endl;
    return 0;

}
