#include <iostream>
#include <fstream>

#include <time.h>
#include <sys/time.h>

#include <ddp-actuator-solver/ddpsolver.hh>
#include "romeosimpleactuator.hh"
#include "costfunctionromeoactuator.hh"

using namespace std;
using namespace Eigen;

int main()
{
    DDPSolver<double,4,1>::stateVec_t xinit,xDes,x;
    DDPSolver<double,4,1>::commandVec_t u;

    u << 1.0;

    xinit << -1.0,0.0,-100.0,0.0;
    xDes << 1.0,0.0,0.0,0.0;

    x = xinit;

    double dt=1e-3;
    unsigned int N=3000;
    DDPSolver<double,4,1>::stateVecTab_t xList;
    DDPSolver<double,4,1>::commandVecTab_t uList;

    xList .resize(N);
    uList.resize(N);

    RomeoSimpleActuator romeoActuatorModel(dt);

    for(unsigned int i=0;i<N;i++)
    {
        xList[i] = x;
        uList[i] = u;
        x = romeoActuatorModel.computeNextState(dt,x,u);
    }

    ofstream fichier("results.csv",ios::out | ios::trunc);
    if(fichier)
    {
        fichier << "tau,tauDot,q,qDot,u" << endl;
        for(unsigned int i=0;i<N;i++)
	  fichier << xList[i](0,0) << "," << xList[i](1,0)
		  << "," << xList[i](2,0) << ","
		  << xList[i](3,0) << "," << uList[i](0,0)
		  << endl;
        fichier.close();
    }
    else
        cerr << "erreur ouverte fichier" << endl;
    return 0;

}
