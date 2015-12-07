#include <iostream>
#include <fstream>

#include "config.h"

#include "ilqrsolver.h"
#include "costfunctionpneumaticarmelbow.h"
#include "pneumaticarmnonlinearmodel.h"
#include <time.h>
#include <sys/time.h>


using namespace std;
using namespace Eigen;

int main()
{
    struct timeval tbegin,tend;
    double texec=0.0;
    stateVec_t xinit,xDes;

    //xinit << 0.0,0.0,0.0,0.0;
    //xDes << 1.0,0.0,0.0,0.0;
    //xinit << 0.0,   0.0,    0.0,    4.0*1e5;
    //xDes << 1.0,    0.0,    2.0*1e5,    2.0*1e5;
    xinit << 0.0,0.0,0.0,4.0*1e5;
    xDes << 1.0,0.0,2.0*1e5,    2.0*1e5;
    
    unsigned int T = 3000;
    double dt=5e-3;
    unsigned int iterMax = 20;
    double stopCrit = 1e-5;
    stateVec_t* xList;
    commandVec_t* uList;
    ILQRSolver::traj lastTraj;
   
    PneumaticarmNonlinearModel pneumaticarmModel(dt);
    CostFunctionPneumaticarmElbow costPneumatic;
    ILQRSolver testSolverRomeoActuator(pneumaticarmModel,costPneumatic);

    //RomeoSimpleActuator romeoActuatorModel(dt);
    //RomeoLinearActuator romeoLinearModel(dt);
    //CostFunctionRomeoActuator costRomeoActuator;
    //ILQRSolver testSolverRomeoActuator(romeoActuatorModel,costRomeoActuator);
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





    ofstream fichier("/home/gkharish/softdev/DDP/cpp/src/results.csv",ios::out | ios::trunc);
    if(fichier)
    {
        fichier << "angular_position,angular_speed,angular_acceleration,u" << endl;
        for(int i=0;i<T;i++) fichier << xList[i](0,0) << "," << xList[i](1,0) << "," << xList[i](2,0) << "," << xList[i](3,0) << "," << uList[i](0,0) << "," << uList[i](1,0) << endl;
        fichier << xList[T](0,0) << "," << xList[T](1,0) << "," << xList[T](2,0) << "," << xList[T](3,0)  << "," << 0.0 << "," << 0.0 << endl;
        fichier.close();
    }
    else
        cerr << "erreur ouverte fichier" << endl;
    return 0;

}
