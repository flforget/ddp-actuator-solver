#ifndef DYNAMICMODEL_H
#define DYNAMICMODEL_H

#include <Eigen/Core>

using namespace Eigen;

class DynamicModel
{
// constructors //
public:
    DynamicModel();

// attributes //
public:
private:
    static unsigned int stateNb;
    static unsigned int commandNb;

protected:

// methods //
public:
    virtual void computeNextState(unsigned int dt,unsigned int X,unsigned int U);
    virtual void computeAllModelDeriv(unsigned int dt,unsigned int X,unsigned int U);
    virtual unsigned int getStateNb();
    virtual unsigned int getCommandNb();
private:
protected:
};

#endif // DYNAMICMODEL_H
