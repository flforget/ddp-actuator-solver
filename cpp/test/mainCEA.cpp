#include <iostream>
#include <fstream>

#include "config.h"

#include "ilqrsolver.h"
#include "ceaactuator.h"
#include "costcea.h"

#include <time.h>
#include <sys/time.h>


using namespace std;
using namespace Eigen;

int main()
{
    struct timeval tbegin,tend;
    double texec=0.0;

    ofstream fichier("resultsCEA.csv",ios::out | ios::trunc);
    if(fichier) fichier << "thetaDot,qDot,diff,u" << endl;

    ILQRSolver<double,4,2>::stateVec_t xinit,xDes,x;
    ILQRSolver<double,4,2>::commandVec_t u;

    xinit << 0.0,0.0,0.0,0.0;
    xDes << 1.0,0.0,0.0,0.0;

    unsigned int T = 300;
    double dt=1e-3;
    unsigned int iterMax = 100;
    double stopCrit = 1e-5;
    ILQRSolver<double,4,2>::stateVecTab_t xList;
    ILQRSolver<double,4,2>::commandVecTab_t uList;
    ILQRSolver<double,4,2>::traj lastTraj;

    CeaActuator romeoActuatorModel(dt);
    CostCea costRomeoActuator;
    ILQRSolver<double,4,2> testSolverRomeoActuator(romeoActuatorModel,costRomeoActuator,ENABLE_FULLDDP,DISABLE_QPBOX);
    testSolverRomeoActuator.FirstInitSolver(xinit,xDes,T,dt,iterMax,stopCrit);

    int N = 1;
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

    /*xList.resize(T+1);
    uList.resize(T);
    double err=0.0;
    double err_int=0.0;

    xinit << 0.0,0.0,0.0,0.0;
    xDes << 0.0,0.0,0.0,0.0;
    u << 0.0,0.0;
    x = xinit;
    xList[0] = x;
    for(int i=0;i<T;i++)
    {
        err = (1.0-x(0));
        err_int +=dt*err;
        u << 10.0*err,0.05;
        if(u(0)>5.0) u(0) = 5.0;
        if(u(0)<-5.0) u(0) = -5.0;
        x = romeoActuatorModel.computeNextState(dt,x,xDes,u);
        xList[i+1] = x;
        uList[i] = u;
    }*/

    if(fichier)
    {
        for(int i=0;i<T;i++) fichier << xList[i](0,0) << "," << xList[i](1,0) << "," << xList[i](2,0) << "," << xList[i](3,0) << "," << uList[i](0,0)  << "," << uList[i](1,0) << endl;
        fichier << xList[T](0,0) << "," << xList[T](1,0) << "," << xList[T](2,0) << "," << xList[T](3,0) << "," << uList[T-1](0,0) << "," << uList[T-1](1,0)  << endl;
        fichier.close();
    }
    else
        cerr << "erreur ouverte fichier" << endl;
    return 0;

}
