#ifndef CONFIG_H
#define CONFIG_H

#include <Eigen/Dense>

#define stateSize 4
#define commandSize 2

// typedef for stateSize types
typedef Eigen::Matrix<double,stateSize,1> stateVec_t;                       // stateSize x 1
typedef Eigen::Matrix<double,1,stateSize> stateVecTrans_t;                  // 1 x stateSize
typedef Eigen::Matrix<double,stateSize,stateSize> stateMat_t;               // stateSize x stateSize
typedef Eigen::Matrix<double,stateSize,stateSize> stateTens_t[stateSize];   // stateSize x stateSize x stateSize

// typedef for commandSize types
typedef Eigen::Matrix<double,commandSize,1> commandVec_t;                           // commandSize x 1
typedef Eigen::Matrix<double,1,commandSize> commandVecTrans_t;                      // 1 x commandSize
typedef Eigen::Matrix<double,commandSize,commandSize> commandMat_t;                 // commandSize x commandSize
typedef Eigen::Matrix<double,commandSize,commandSize> commandTens_t[commandSize];   // stateSize x commandSize x commandSize



// typedef for mixed stateSize and commandSize types
typedef Eigen::Matrix<double,stateSize,commandSize> stateR_commandC_t;                          // stateSize x commandSize
typedef Eigen::Matrix<double,stateSize,commandSize> stateR_commandC_stateD_t[stateSize];        // stateSize x commandSize x stateSize
typedef Eigen::Matrix<double,stateSize,commandSize> stateR_commandC_commandD_t[commandSize];    // stateSize x commandSize x commandSize
typedef Eigen::Matrix<double,commandSize,stateSize> commandR_stateC_t;                          // commandSize x stateSize
typedef Eigen::Matrix<double,commandSize,stateSize> commandR_stateC_stateD_t[stateSize];        // commandSize x stateSize x stateSize
typedef Eigen::Matrix<double,commandSize,stateSize> commandR_stateC_commandD_t[commandSize];    // commandSize x stateSize x commandSize
typedef Eigen::Matrix<double,stateSize,stateSize> stateR_stateC_commandD_t[commandSize];    // stateSize x stateSize x commandSize
typedef Eigen::Matrix<double,commandSize,commandSize> commandR_commandC_stateD_t[stateSize];    // commandSize x commandSize x stateSize


#endif // CONFIG_H
