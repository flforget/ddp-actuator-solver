#define stateSize 4
#define commandSize 1

#include <iostream>
#include "ilqrsolver.h"
#include "romeosimpleactuator.h"
#include "costfunctionromeoactuator.h"
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

typedef Matrix<double,4,1> V4;

int main()
{
    DynamicModel testModel;
    CostFunction testCostFun;
    RomeoSimpleActuator romeoActuatorModel;
    CostFunctionRomeoActuator costRomeoActuator;
    ILQRSolver testSolver(testModel,testCostFun);
    return 0;
}

