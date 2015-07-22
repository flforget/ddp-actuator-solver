#define stateSize 4
#define commandSize 1

#include <iostream>
#include "ilqrsolver.h"
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

int main()
{
    DynamicModel testModel;
    CostFunction testCostFun;
    ILQRSolver testSolver(testModel,testCostFun);
    cout << testModel.getCommandNb() << endl;
    return 0;
}

