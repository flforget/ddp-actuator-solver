#ifndef COSTFUNCTIONROMEOACTUATOR_H
#define COSTFUNCTIONROMEOACTUATOR_H

#include <Eigen/Core>

using namespace Eigen;

typedef Matrix<double,4,4> M4;
typedef Matrix<double,4,1> V4;
typedef Matrix<double,1,4> V4T;

class CostFunctionRomeoActuator
{
public:
    CostFunctionRomeoActuator();
private:
    M4 Q;
    double R;
    V4 lx;
    M4 lxx;
    double lu,luu;
    V4T lux;
    V4 lxu;
protected:
    // attributes //
public:
private:

protected:
    // methods //
public:
    void computeAllCostDeriv(V4& X, V4& Xdes, double& U);
    void commuteFinalCostDeriv(V4& X, V4& Xdes);
private:
protected:
    // accessors //
public:
    V4 getlx();
    M4 getlxx();
    double getlu();
    double getluu();
    V4T getlux();
    V4 getlxu();
};

#endif // COSTFUNCTIONROMEOACTUATOR_H
