#include <iostream>
#include <fstream>

#include "config.h"

#include "ilqrsolver.h"
//#include "romeosimpleactuator.h"
//#include "romeolinearactuator.h"
//#include "costfunctionromeoactuator.h"
#include "costfunctionpneumaticarmelbow.h"
//#include "pneumaticarmelbowlinear.h"
#include "pneumaticarmnonlinearmodel.h"
//#include "pneumaticarm_model.h"
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

    xinit << -1.0,0.0, -2.0e5, 2.0e5;
    //xDes << 1.0,0.0,0.0,0.0;

    unsigned int T = 10;
    unsigned int M = 1000;
    double dt=5e-3;
    unsigned int iterMax = 20;
    double stopCrit = 1e-3;
    stateVec_t* xList;
    commandVec_t* uList;
    ILQRSolver::traj lastTraj;

   /* RomeoSimpleActuator romeoActuatorModel(dt);
    RomeoLinearActuator romeoLinearModel(dt);
    CostFunctionRomeoActuator costRomeoActuator;
    ILQRSolver testSolverRomeoActuator(romeoActuatorModel,costRomeoActuator);*/
   
    PneumaticarmNonlinearModel pneumaticarmModel(dt);
    //PneumaticarmElbowPieceLinear pneumaticPieceLinearModel(dt);
    //CostFunctionRomeoActuator costRomeoActuator;
    CostFunctionPneumaticarmElbow costPneumatic;
    ILQRSolver testSolver(pneumaticarmModel,costPneumatic);


    ofstream fichier("resultsMPC.csv",ios::out | ios::trunc);
    if(!fichier)
    {
        cerr << "erreur fichier ! " << endl;
        return 1;
    }
    fichier << T << "," << M << endl;
    fichier << "Theta,thetaDot,P1,P2,u1,u2" << endl;


    testSolver.FirstInitSolver(xinit,T,dt,iterMax,stopCrit);

    gettimeofday(&tbegin,NULL);
    for(int i=0;i<M;i++)
    {
        testSolver.initSolver(xinit);
        testSolver.solveTrajectory();
        lastTraj = testSolver.getLastSolvedTrajectory();
        xList = lastTraj.xList;
        uList = lastTraj.uList;
        xinit = xList[1];
        /*for(int j=0;j<T;j++) fichier << xList[j](0,0) << "," << xList[j](1,0) << "," << xList[j](2,0) << "," << xList[j](3,0) << "," << uList[j](0,0) << endl;*/
        fichier << xList[1](0,0) << "," << xList[1](1,0) << "," << xList[1](2,0) << "," << xList[1](3,0) << "," << uList[0](0.0) << "," << uList[0](1,0) << endl;
    }
    gettimeofday(&tend,NULL);


    texec=((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;
    texec = (double)(tend.tv_usec - tbegin.tv_usec);

    cout << "temps d'execution total du solveur ";
    cout << texec/1000000.0 << endl;
    cout << "temps d'execution par pas de MPC ";
    cout << texec/(T*1000000) << endl;

//    fichier.close();

 return 0;

}

