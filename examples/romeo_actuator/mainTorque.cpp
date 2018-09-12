#include <iostream>
#include <fstream>

#include <ddp-actuator-solver/ddpsolver.hh>
#include "romeotorqueactuator.hh"
#include "costfunctionromeoactuator.hh"

#include <time.h>
#include <sys/time.h>

using namespace std;
using namespace Eigen;

int main()
{
  struct timeval tbegin, tend;
  double texec = 0.0;
  DDPSolver<double, 4, 1>::stateVec_t xinit, xDes, x;
  DDPSolver<double, 4, 1>::commandVec_t u;

  xinit << 0.0, 0.0, 0.0, 0.0;
  xDes << 0.1, 0.0, 0.0, 0.0;

  unsigned int T = 50;
  double dt = 1e-3;
  unsigned int iterMax = 100;
  double stopCrit = 1e-5;
  DDPSolver<double, 4, 1>::stateVecTab_t xList;
  DDPSolver<double, 4, 1>::commandVecTab_t uList;
  DDPSolver<double, 4, 1>::traj lastTraj;

  RomeoTorqueActuator romeoActuatorModel(dt);
  CostFunctionRomeoActuator costRomeoActuator;
  DDPSolver<double, 4, 1> testSolverRomeoActuator(romeoActuatorModel,
      costRomeoActuator, DISABLE_FULLDDP, ENABLE_QPBOX);
  testSolverRomeoActuator.FirstInitSolver(xinit, xDes, T, dt, iterMax, stopCrit);

  int N = 100;
  gettimeofday(&tbegin, NULL);
  for (int i = 0; i < N; i++) testSolverRomeoActuator.solveTrajectory();
  gettimeofday(&tend, NULL);

  lastTraj = testSolverRomeoActuator.getLastSolvedTrajectory();
  xList = lastTraj.xList;
  uList = lastTraj.uList;
  unsigned int iter = lastTraj.iter;

  texec = ((double)(1000 * (tend.tv_sec - tbegin.tv_sec) + ((
                      tend.tv_usec - tbegin.tv_usec) / 1000))) / 1000.;
  texec /= N;

  cout << endl;
  cout << "temps d'execution total du solveur ";
  cout << texec << endl;
  cout << "temps d'execution par pas de temps ";
  cout << texec / T << endl;
  cout << "Nombre d'itérations : " << iter << endl;


  ofstream fichier("results.csv", ios::out | ios::trunc);
  if (fichier)
  {
    fichier << "tau,tauDot,q,qDot,u" << endl;
    for (int i = 0; i < T; i++)
    {
      fichier << xList[i](0, 0) << "," << xList[i](1, 0) << "," 
              << xList[i](2, 0) << "," << xList[i](3, 0) << "," 
              << uList[i](0, 0) << endl;
    }
    fichier << xList[T](0, 0) << "," << xList[T](1, 0) << "," 
            << xList[T](2, 0) << "," << xList[T](3, 0) << "," 
            << uList[T - 1](0, 0) << endl;
    fichier.close();
  }
  else
    cerr << "erreur ouverte fichier" << endl;
  return 0;

}
