// Copydight (c) 2015 LAAS_CNRS //
// Author: Ganesh Kumar //


/* This is a program to model the pneumatic muscle based joint of the Pneumatic 7R arm */

#ifndef PNEUMATICARMMODEL_HH
#define PNEUMATICARMMODEL_HH

//#include <Eigen/Dense>
//#include <Eigen/Core>
#include <math.h>
#include <vector>
#define GRAVITY 9.81
#define PI 3.14159265
#include "config.h"

#include "dynamicmodel.h"
#include <Eigen/Dense>
using namespace std;
//using namespace Eigen;

class PneumaticarmModel
{
 protected:
            double length_;
            double mass_;
            double friction_;
            double Torque_, TorqueDes_;
            float pressure_muscle1_, pressure_muscle2_, pressure_musclebase_;
            double lo, alphao, k,ro,R,a,b,emax,lb,lt,epsb,epst;
            double time_constant1, time_constant2;

            // Joint parameters 
            double m;
            double link_l;
            double g;
            double I;
            double fv;


            int nDOF_;
           unsigned int n_;
            std::vector<double> state_vector_;
            std::vector<double> state_derivative_;
            std::vector<double> control_vector_;
        public:
                /// Constructor
                PneumaticarmModel();   
                virtual  ~PneumaticarmModel();
                void setProblemDimension (int n);
                void setParameters (void);
                //void setpidcoeff(int p, int i, int d);
                void computeStateDerivative (double time);
                void integrateRK4 (double time, double timeStep);
                double InverseModel(vector<double>& reference);
                // vector<double> integrateEuler (double time, double timeStep);
                void Set_ControlVector(double value, unsigned int idx);
                void Set_StateVector(double value, unsigned int idx);
                double Get_StateVector(unsigned idx);
                double Get_ControlVector(unsigned int idx);
                double Get_Torque();
                double Get_TorqueDes();
                //VectorXd getControl (VectorXd statevector, double reference_position, double position);
   };

    

    
#endif

