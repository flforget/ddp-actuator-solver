#include <iostream>

#include "config.h"

#include "ilqrsolver.h"
#include "romeosimpleactuator.h"
#include "costfunctionromeoactuator.h"
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

int main()
{
    DynamicModel testModel;
    CostFunction testCostFun;
    RomeoSimpleActuator romeoActuatorModel;
    CostFunctionRomeoActuator costRomeoActuator;
    ILQRSolver testSolver(testModel,testCostFun);
    ILQRSolver testSolverRomeoActuator(romeoActuatorModel,costRomeoActuator);

    return 0;
}

