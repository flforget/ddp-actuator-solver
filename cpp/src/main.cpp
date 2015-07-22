#include <iostream>
#include "ilqrsolver.h"
#include "romeosimpleactuator.h"
#include <Eigen/Core>

using namespace std;

int main()
{
    RomeoSimpleActuator romeoActuatorModel;
    CostFunction costFunction;
    ILQRSolver iLQRSolver(romeoActuatorModel,costFunction);

    Eigen::VectorXf xInit(4);
    Eigen::VectorXf xDes(4);

    Eigen::VectorXf xList[2];



    xDes(0) = 1.0;

    xList[0] = xInit;
    xList[1] = xDes;

    cout << xList[0]<< endl;
    cout << xList[1] << endl;

    cout << "done" << endl;
}

