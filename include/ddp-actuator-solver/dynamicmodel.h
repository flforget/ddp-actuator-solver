#ifndef DYNAMICMODEL_H
#define DYNAMICMODEL_H

#include <Eigen/Core>

template<typename precision,int stateSize,int commandSize>
class DynamicModel
{
public:
    typedef Eigen::Matrix<precision,stateSize,1> stateVec_t;                  // 1 x stateSize
    typedef Eigen::Matrix<precision,1,stateSize> stateVecTrans_t;                  // 1 x stateSize
    typedef Eigen::Matrix<precision,stateSize,stateSize> stateMat_t;               // stateSize x stateSize
    typedef Eigen::Matrix<precision,stateSize,stateSize> stateTens_t[stateSize];   // stateSize x stateSize x stateSize

    // typedef for commandSize types
    typedef Eigen::Matrix<precision,commandSize,1> commandVec_t;                           // commandSize x 1
    typedef Eigen::Matrix<precision,1,commandSize> commandVecTrans_t;                      // 1 x commandSize
    typedef Eigen::Matrix<precision,commandSize,commandSize> commandMat_t;                 // commandSize x commandSize
    typedef Eigen::Matrix<precision,commandSize,commandSize> commandTens_t[commandSize];   // stateSize x commandSize x commandSize



    // typedef for mixed stateSize and commandSize types
    typedef Eigen::Matrix<precision,stateSize,commandSize> stateR_commandC_t;                          // stateSize x commandSize
    typedef Eigen::Matrix<precision,stateSize,commandSize> stateR_commandC_stateD_t[stateSize];        // stateSize x commandSize x stateSize
    typedef Eigen::Matrix<precision,stateSize,commandSize> stateR_commandC_commandD_t[commandSize];    // stateSize x commandSize x commandSize
    typedef Eigen::Matrix<precision,commandSize,stateSize> commandR_stateC_t;                          // commandSize x stateSize
    typedef Eigen::Matrix<precision,commandSize,stateSize> commandR_stateC_stateD_t[stateSize];        // commandSize x stateSize x stateSize
    typedef Eigen::Matrix<precision,commandSize,stateSize> commandR_stateC_commandD_t[commandSize];    // commandSize x stateSize x commandSize
    typedef Eigen::Matrix<precision,stateSize,stateSize> stateR_stateC_commandD_t[commandSize];    // stateSize x stateSize x commandSize
    typedef Eigen::Matrix<precision,commandSize,commandSize> commandR_commandC_stateD_t[stateSize];    // commandSize x commandSize x stateSize

// constructors //
public:
// attributes //
public:
protected:
    unsigned int stateNb;
    unsigned int commandNb;
    double dt;

    commandVec_t lowerCommandBounds;
    commandVec_t upperCommandBounds;

    stateMat_t fx;
    stateTens_t fxx;
    stateR_commandC_t fu;
    stateR_commandC_commandD_t fuu;
    stateR_stateC_commandD_t fxu;
    stateR_commandC_stateD_t fux;
public:


protected:


// methods //
public:
    virtual stateVec_t computeNextState(double& dt, const stateVec_t& X,const commandVec_t& U)=0;
    virtual void computeAllModelDeriv(double& dt, const stateVec_t& X,const commandVec_t& U)=0;
    virtual stateMat_t computeTensorContxx(const stateVec_t& nextVx)=0;
    virtual commandMat_t computeTensorContuu(const stateVec_t& nextVx)=0;
    virtual commandR_stateC_t computeTensorContux(const stateVec_t& nextVx)=0;
private:
protected:
    // accessors //
public:
    unsigned int getStateNb()               {return stateNb;}
    unsigned int getCommandNb()             {return commandNb;}
    commandVec_t& getLowerCommandBounds()   {return lowerCommandBounds;}
    commandVec_t& getUpperCommandBounds()   {return upperCommandBounds;}
    stateMat_t& getfx()                     {return fx;}
    stateTens_t& getfxx()                   {return fxx;}
    stateR_commandC_t &getfu()              {return fu;}
    stateR_commandC_commandD_t& getfuu()    {return fuu;}
    stateR_stateC_commandD_t& getfxu()      {return fxu;}
    stateR_commandC_stateD_t& getfux()      {return fux;}
};

#endif // DYNAMICMODEL_H
