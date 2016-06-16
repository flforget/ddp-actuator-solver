
// Authors: Ganesh Kumar


/* ******** It is a trial program for modelling a Pneumatic muscle and 
   ******** controlling it in closeloop under xenomai realtime kernel ********/

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <iostream>
#include <cmath>
//#include <native/task.h>
//#include <native/timer.h>
#include <time.h>
#include <string.h>
#include <sstream>
//#include <debug.hh>

#include "pneumaticarm_model.h"

double pi1 =3.14;
PneumaticarmModel::PneumaticarmModel()
{
    state_vector_.resize(4);
    state_derivative_.resize(4);
    control_vector_.resize(2);

}

/* Setting Number of Joints or degree of freedom*/
void PneumaticarmModel::setProblemDimension (int n)
{
    nDOF_ = n;
}
        
/* Initialization or setting the parameters */
void PneumaticarmModel::setParameters (void)
{
            
            
    length_ = 1.0; // m
    mass_ = 1.0;   // kg
    friction_ = 0.1; // kg/s
    pressure_musclebase_ = 2.5; //bar
    n_ = 4; // Number of states in the state spase model
    /* M=1; //Mass (kg)
    K = 30000; //stiffness 
    L = 1; //Length of the rod
    g = 9.8;
    I = 1; //Inertia of the primary motor
    J = 1; */ // Inertia of the other m*/
}
        
 /*PAM system dynamics and Compute state derivatives*/
        
void PneumaticarmModel::computeStateDerivative(double time)
{
    //VectorXd state_derivative(statevector.size());
    //Parameters Muscles
    //double Tmax, fk,fs, a, b, K0, K1, K2, P_m1, P_m2;        
    lo = 0.185;
    alphao = 20.0*PI/180;
    //double epsilono = 0.15;
    k = 1.25;
    ro = 0.0085;
    // Parameters Joint
    R = 0.015;
    m = 2.6;
    link_l = 0.32;
    g = 9.81;
    time_constant1 = 0.18;
    time_constant2 = 0.13;
    //double velocity_constant = 0.15;
    I = m*link_l*link_l/3; //0.0036;
    fv = 0.25;
    double F1, F2, P1, P2, Tc1,Tc2, u1,u2;
    
    P1 = state_vector_[2];
    P2 = state_vector_[3];
    u1 = control_vector_[0];
    u2 = control_vector_[1];
    Tc1 = time_constant1;
    Tc2 = time_constant2;
    a = 3/pow(tan(alphao), 2);
    b = 1/pow(sin(alphao), 2);
    emax = (1/k)*(1 - sqrt(b/a));
    double lreal = lo - R*0.25;
    lb = lreal- R*state_vector_[0];
    epsb = (1-(lb/lo));
    lt = lo*(1-emax) + R*state_vector_[0];
    epst = (1-(lt/lo));
    F1 =  pi1*pow(ro,2)*P1*(a*pow((1-k*epsb),2) - b);
    F2 =  pi1*pow(ro,2)*P2*(a*pow((1-k*epst),2) - b);
    //F2max = 1*pi1*ro^2*4*1e5*(a*(1-k*emax)^2 - b);
    Torque_ = (F1 -F2 )*R;
    state_derivative_[0] = state_vector_[1];
    state_derivative_[1] =  ((F1 -F2 )*R  - fv*state_vector_[1] - m*g*0.5*link_l*sin(state_vector_[0]))/I;
    state_derivative_[2] = (-P1/Tc1) + (u1/Tc1);
    state_derivative_[3] = (-P2/Tc2) + (u2/Tc2);           
     //P_m1 = 0.675;
    //P_m2 = 4.0;
    ///////////////////////////////////////////////////////////////////////////
/* Parameters for the muscles
lo = 0.185;
alphao = 23.0*pi1/180;
%epsilono = 0.15;
%emax = 0.2983;
k = 1.25;
ro = 0.009;
R = 0.015;

a = 3/(tan(alphao))^2;
b = 1/(sin(alphao))^2;
emax = (1/k)*(1 - sqrt(b/a));

lb = lo- R*theta;
epsb = (1-(lb/lo));
lt = lo*(1-emax) + R*theta;
epst = (1-(lt/lo));

%% Parameters of Joint
m = 2.6;
link_l = 0.32;
g =9.81;
I = m*(link_l^2)/3;
fv = 0.25;


F1 =  pi1*ro^2*P1*1e5*(a*(1-k*epsb)^2 - b);
F2 =  pi1*ro^2*P2*1e5*(a*(1-k*epst)^2 - b);
F2max = 1*pi1*ro^2*4*1e5*(a*(1-k*emax)^2 - b);
T = (F1 -F2 )*R;
F = [F1 F2 T];
jointstate_deriv(1) = theta_dot; %joint_state(2);
jointstate_deriv(2) = ((F1 -F2 )*R  - fv*theta_dot - m*g*0.5*link_l*sin(theta))/I;*/
    ///////////////////////////////////////////////////////////////////////////


   // ODEBUGL("State derivative: "<< state_derivative_[0],0);
}
 
        
/* Numerical Integrator Rungee Kutta */
void PneumaticarmModel::integrateRK4 (double t, double h)
{
    vector<double> st1, st2, st3, st4, state_temp_;
    st1.resize(n_);
    st2.resize(n_);
    st3.resize(n_);
    st4.resize(n_);
    state_temp_.resize(n_);
    for (unsigned int i =0; i <n_; i++)
    {
        state_temp_[i] = state_vector_[i];
    }
    computeStateDerivative (t);
    for (unsigned int i =0; i <n_; i++)
    {
        st1[i] = state_derivative_[i];
        state_vector_[i] = state_temp_[i] + 0.5*h*st1[i];
    }
   
    computeStateDerivative (t + (0.5 * h));
    for (unsigned int i =0; i <n_; i++)
    {
        st2[i] = state_derivative_[i];
        state_vector_[i] = state_temp_[i] + 0.5*h*st2[i];
    }
        
   computeStateDerivative (t + (0.5 * h));
   for (unsigned int i =0; i <n_; i++)
   {
        st3[i] = state_derivative_[i];
        state_vector_[i] = state_temp_[i] + h*st3[i];
   }
   
   computeStateDerivative (t + h);
   for (unsigned int i =0; i <n_; i++)
        st4[i] = state_derivative_[i];
  
  
   for (unsigned int i =0; i <n_; i++)
       state_vector_[i]= state_temp_[i] + ( (1/6.0) * h * (st1[i] + 2.0*st2[i] + 2.0*st3[i] + st4[i]) );
   //ODEBUGL("State vector: " << state_vector_[0],0);
}
        
/* Numerical Integrator Euler */
/*VectorXd PneumaticarmModel::integrateEuler (double t, VectorXd state, VectorXd u, double h)
{
    VectorXd st = computeStateDerivative (t, state, u);
            
    VectorXd stNew = state + h*st;
    return (stNew);
}*/
double  PneumaticarmModel::InverseModel (vector<double>& reference)
{
    //Parameters Muscles
    //double Tmax, fk,fs, a, b, K0, K1, K2, P_m1, P_m2;        
    double lo = 0.185;
    double alphao = 20.0*PI/180;
    //double epsilono = 0.15;
    double k = 1.25;
    double ro = 0.0085;
    // Parameters Joint
    double R = 0.015;
    double m = 2.6;
    double link_l = 0.32;
    double g = 9.81;
    //double time_constant = 0.1;
    //double velocity_constant = 0.15;
    double I = m*link_l*link_l/3; //0.0036;
    double fv = 0.25;
    
    double theta, theta_dot, theta_dot2;
    double a, b, t1, t2, Pmax, tor1, tor2, P_meanDes, Fmax, emax;
    theta = reference[0];//%(t-1)*5*pi/180;         %ref_traj(1);
    theta_dot = reference[1];//%5*pi1/180;     %ref_traj(2);
    theta_dot2 = reference[2];
    //theta_dot3 = reference[3];
    //theta_dot4 = reference[4];
    double lreal = lo -R*0.25;
    Pmax = 4.0*1e5;
    a = 3/pow(tan(alphao), 2);
    b = 1/pow(sin(alphao), 2);
    emax = (1/k)*(1 - sqrt(b/a));
    Fmax = (pi1*pow(ro,2))*(a-b)*Pmax;
    t1 = R*theta/(lreal*emax);
    t2 = (I*theta_dot2 + fv*theta_dot + m*g*link_l*0.5*sin(theta))/(R*Fmax);
   
    P_meanDes = Pmax*(t1 + t2);
    tor1 = P_meanDes/Pmax;
    tor2 = R*theta/(lo*emax);
    TorqueDes_ = R*Fmax*(tor1 -tor2);
    return(P_meanDes*1e-5);

   /* t1dot = R*theta_dot/(lo*emax);
    t2dot = (I*theta_dot3 + fv*theta_dot2 + m*g*link_l*0.5*cos(theta))/(R*Fmax);

    P_real_dot = Pmax*(t1dot + t2dot);
    
    t1dot2 = R*theta_dot2/(lo*emax);
    t2dot2 = (I*theta_dot4 + fv*theta_dot3 - m*g*link_l*0.5*sin(theta))/(R*Fmax);
    P_real_dot2 = Pmax*(t1dot2 + t2dot2);

    P1 = P1o + P_real;
    P2 = P2o - P_real;

    P1dot = P_real_dot;
    P2dot = -P_real_dot;

    P1dot2 = P_real_dot2;
    P2dot2 = -P_real_dot2;

    P = [P1 P2];
    Pdot = [P1dot P2dot];
    Pdot2 = [P1dot2 P2dot2];*/
}
       

void PneumaticarmModel::Set_ControlVector (double value, unsigned int idx)

{
    control_vector_[idx] = value;
    //ODEBUGL("Control vector is set" << control_vector_[idx],0);
}

double PneumaticarmModel::Get_ControlVector(unsigned int idx)
{
    return(control_vector_[idx]);
}

double PneumaticarmModel::Get_StateVector(unsigned int idx)
{
    return(state_vector_[idx]);
}

void PneumaticarmModel::Set_StateVector(double value, unsigned int idx)
{
    state_vector_[idx] = value;
}
double PneumaticarmModel::Get_Torque()
{
   return(Torque_);
}
double PneumaticarmModel::Get_TorqueDes()
{
   return(TorqueDes_);
}
PneumaticarmModel::~PneumaticarmModel()
{
}
