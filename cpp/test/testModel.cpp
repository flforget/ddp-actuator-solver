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
    stateVec_t xinit,xDes,x;
    commandVec_t u;

    u << 2.0,2.0;

    xinit << 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
    xDes << 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;

    x = xinit;

    unsigned int T = 50;
    double dt=3e-5;
    unsigned int iterMax = 50;
    double stopCrit = 1e-5;
    stateVecTab_t xList;
    commandVecTab_t uList;
    ILQRSolver::traj lastTraj;

    PneumaticarmNonlinearModel model(dt);
    CostFunctionPneumaticarmElbow cost;

    //cout << romeoActuatorModel.getCommandNb() << endl;

    ofstream fichier("results.csv",ios::out | ios::trunc);
    if(fichier)
    {
        fichier << "tau,tauDot,q,qDot,u" << endl;
    }

    for(unsigned int i=0;i<100;i++)
    {
        x = model.computeNextState(dt,x,xDes,u);
        cout << x << endl<<"-"<<endl;
        if(fichier)
        {
            fichier << x(0,0) << "," << x(1,0) << "," << x(2,0) << "," << x(3,0) << "," << u(0,0) << endl;
        }
        else
            cerr << "erreur ouverte fichier" << endl;

    }

    fichier.close();
    return 0;
    /*ILQRSolver testSolverRomeoActuator(model,cost,ENABLE_FULLDDP,DISABLE_QPBOX);
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
    cout << "Nombre d'itérations : " << iter << endl;*/


    return 0;

}
