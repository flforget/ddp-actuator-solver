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
  cout << endl;
  struct timeval tbegin, tend;
  double texec = 0.0;
  DDPSolver<double, 4, 1>::stateVec_t xinit, xDes;

  xinit << 0.0, 0.0, 0.0, 0.0;
  xDes << 1.0, 0.0, 0.0, 0.0;

  unsigned int T = 100;
  unsigned int M = 3000;
  double dt = 1e-3;
  unsigned int iterMax = 100;
  double stopCrit = 0.01;
  DDPSolver<double, 4, 1>::stateVecTab_t xList;
  DDPSolver<double, 4, 1>::commandVecTab_t uList;
  DDPSolver<double, 4, 1>::traj lastTraj;

  srand(static_cast<unsigned int>(time(NULL)));

  RomeoSimpleActuator romeoActuatorModel(dt);
  RomeoSimpleActuator romeoNoisyModel(dt, 1);
  CostFunctionRomeoActuator costRomeoActuator;
  DDPSolver<double, 4, 1> testSolverRomeoActuator(romeoActuatorModel,
      costRomeoActuator, DISABLE_FULLDDP, DISABLE_QPBOX);

  ofstream fichier("resultsMPC.csv", ios::out | ios::trunc);
  if (!fichier)
  {
    cerr << "erreur fichier ! " << endl;
    return 1;
  }
  fichier << T << "," << M << endl;
  fichier << "tau,tauDot,q,qDot,u" << endl;

  testSolverRomeoActuator.FirstInitSolver(xinit, xDes, T, dt, iterMax,
      stopCrit);

  gettimeofday(&tbegin, NULL);
  for (unsigned int i = 0; i < M; i++)
  {
    testSolverRomeoActuator.initSolver(xinit, xDes);
    testSolverRomeoActuator.solveTrajectory();
    lastTraj = testSolverRomeoActuator.getLastSolvedTrajectory();
    xList = lastTraj.xList;
    uList = lastTraj.uList;
    xinit = romeoNoisyModel.computeNextState(dt, xinit, uList[0]);

    for (unsigned int j = 0; j < T; j++)
      fichier << xList[j](0, 0) << "," << xList[j](1, 0) << ","
      << xList[j](2, 0) << "," << xList[j](3, 0) << "," << uList[j](0, 0)
      << endl;
    fichier << xList[T](0, 0) << "," << xList[T](1, 0) << "," << xList[T](2, 0)
            << "," << xList[T](3, 0) << "," << 0.0 << endl;
  }
  gettimeofday(&tend, NULL);

  texec = ((double) (1000 * (tend.tv_sec - tbegin.tv_sec)
      + ((tend.tv_usec - tbegin.tv_usec) / 1000))) / 1000.;
  texec = (double) (tend.tv_usec - tbegin.tv_usec);

  cout << "temps d'execution total du solveur ";
  cout << texec / 1000000.0 << endl;
  cout << "temps d'execution par pas de MPC ";
  cout << texec / (T * 1000000) << endl;

  fichier.close();

  return 0;

}
