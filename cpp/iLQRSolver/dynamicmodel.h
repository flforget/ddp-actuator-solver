#ifndef DYNAMICMODEL_H
#define DYNAMICMODEL_H

#include <Eigen/Core>

class DynamicModel
{
// constructors //
public:
    DynamicModel();

// attributes //
public:
private:
    unsigned int stateNb;
    unsigned int commandNb;
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
