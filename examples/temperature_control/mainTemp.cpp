#include <iostream>
#include <fstream>

#include <ddp-actuator-solver/ddpsolver.hh>
#include "dctemp.hh"
#include "costtemp.hh"

#include <time.h>
#include <sys/time.h>


using namespace std;
using namespace Eigen;

int main()
{
  struct timeval tbegin,tend;
  double texec=0.0;
  DDPSolver<double,5,1>::stateVec_t xinit,xDes,x;
  DDPSolver<double,5,1>::commandVec_t u;

  xinit << -1.0,0.0,-100.0,0.0,0.0;
  xDes << 0.5,0.0,0.0,0.0,0.0;

  unsigned int T = 3000;
  double dt=1e-3;
  unsigned int iterMax = 100;
  double stopCrit = 1e-5;
  DDPSolver<double,5,1>::stateVecTab_t xList;
  DDPSolver<double,5,1>::commandVecTab_t uList;
  DDPSolver<double,5,1>::traj lastTraj;

  DCTemp model;
  DCTemp* noisyModel=NULL;
  CostTemp cost;
  DDPSolver<double,5,1> solver(model,cost,DISABLE_FULLDDP,DISABLE_QPBOX);
  solver.FirstInitSolver(xinit,xDes,T,dt,iterMax,stopCrit);

  int N = 100;
  gettimeofday(&tbegin,NULL);
  solver.solveTrajectory();
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


  ofstream fichier1("results1.csv",ios::out | ios::trunc);
  if(fichier1)
    {
      fichier1 << "tau,tauDot,q,qDot,u" << endl;
      x = xinit;
      fichier1 << x(0, 0) << "," << x(1, 0) << "," << x(2, 0) << ","
	       << x(3, 0) << "," << uList[0] << endl;
      for (unsigned i = 1; i < T; i++)
        {
	  x = model.computeNextState(dt, x, uList[i - 1]);
	  fichier1 << x(0, 0) << "," << x(1, 0) << "," << x(2, 0) << ","
		   << x(3, 0) << "," << uList[i - 1] << endl;
        }
      fichier1 << xList[T](0, 0) << "," << xList[T](1, 0) << "," << xList[T](2, 0) << "," << xList[T](3, 0) << ","
	       << uList[T - 1](0, 0) << endl;
      fichier1.close();
    }
  else
    cerr << "erreur ouverte fichier" << endl;

  ofstream fichier2("results2.csv",ios::out | ios::trunc);
  if(fichier2)
    {
      fichier2 << "tau,tauDot,q,qDot,u" << endl;
      fichier2 << T << ',' << 0 << endl;
      for(int j=0;j<0;j++)
        {
	  noisyModel = new DCTemp(1);
	  x = xinit;
	  for (unsigned int i = 1; i < T; i++)
            {
	      x = noisyModel->computeNextState(dt, x, uList[i - 1]);
	      fichier2 << x(0, 0) << "," << x(1, 0) << "," << x(2, 0) << ","
		       << x(3, 0) << "," << uList[i - 1] << endl;
            }
	  fichier2 << xList[T](0, 0) << "," << xList[T](1, 0) << "," << xList[T](2, 0) << "," << xList[T](3, 0) << ","
		   << uList[T - 1](0, 0) << endl;
	  delete noisyModel;
        }
      fichier2.close();
    }
  else
    cerr << "erreur ouverte fichier" << endl;


  return 0;

}
