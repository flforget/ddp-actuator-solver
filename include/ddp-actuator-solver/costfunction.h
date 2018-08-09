#ifndef COSTFUNCTION_H
#define COSTFUNCTION_H

#include <Eigen/Core>

template<typename precision,int stateSize,int commandSize>
class CostFunction
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

public:
private:
protected:
    // attributes //
public:
private:

protected:
    double dt;
    stateVec_t lx;
    stateMat_t lxx;
    commandVec_t lu;
    commandMat_t luu;
    commandR_stateC_t lux;
    stateR_commandC_t lxu;
    // methods //
public:
    virtual void computeAllCostDeriv(const stateVec_t& X,const stateVec_t& Xdes, const commandVec_t& U)=0;
    virtual void computeFinalCostDeriv(const stateVec_t& X,const stateVec_t& Xdes)=0;
private:
protected:
    // accessors //
public:
    stateVec_t& getlx()         {return lx;}
    stateMat_t& getlxx()        {return lxx;}
    commandVec_t& getlu()       {return lu;}
    commandMat_t& getluu()      {return luu;}
    commandR_stateC_t& getlux() {return lux;}
    stateR_commandC_t& getlxu() {return lxu;}
};

#endif // COSTFUNCTION_H
