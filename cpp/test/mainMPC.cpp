#include <iostream>
#include <fstream>

#include "config.h"

#include "ilqrsolver.h"
//#include "romeosimpleactuator.h"
//#include "romeolinearactuator.h"
//#include "costfunctionromeoactuator.h"
#include "costfunctionpneumaticarmelbow.h"
//#include "pneumaticarmelbowlinear.h"
#include "pneumaticarmnonlinearmodel.h"
#include "pneumaticarm_model.h"
#include <time.h>
#include <sys/time.h>


using namespace std;
using namespace Eigen;

double  InverseModel (stateVec_t reference)
{
    // Parameters Muscles
    // double Tmax, fk,fs, a, b, K0, K1, K2, P_m1, P_m2;        
    double pi = 3.14;
    double lo = 0.185;
    double alphao = 20.0*pi/180;
    // double epsilono = 0.15;
    double k = 1.25;
    double ro = 0.0085;
    // Parameters Joint
    double R = 0.015;
    double m = 2.6;
    double link_l = 0.32;
    double g = 9.81;
    // double time_constant = 0.1;
    // double velocity_constant = 0.15;
    double I = m*link_l*link_l/3; //0.0036;
    double fv = 0.25;
    
    double theta, theta_dot, theta_dot2;
    double a, b, t1, t2, Pmax, tor1, tor2, P_meanDes, Fmax, emax;
    theta = reference(0);//%(t-1)*5*pi/180;         %ref_traj(1);
    theta_dot = reference(1);//%5*pi/180;     %ref_traj(2);
    theta_dot2 = reference(2);
    //theta_dot3 = reference[3];
    //theta_dot4 = reference[4];
    double lreal = lo - R*0.25;
    Pmax = 4.0*1e5;
    a = 3/pow(tan(alphao), 2);
    b = 1/pow(sin(alphao), 2);
    emax = (1/k)*(1 - sqrt(b/a));
    Fmax = (pi*pow(ro,2))*(a-b)*Pmax;
    t1 = R*theta/(lreal*emax);
    t2 = (I*theta_dot2 + fv*theta_dot + m*g*link_l*0.5*sin(theta))/(R*Fmax);
   
    P_meanDes = Pmax*(t1 + t2);
    tor1 = P_meanDes/Pmax;
    tor2 = R*theta/(lo*emax);
    //TorqueDes_ = R*Fmax*(tor1 -tor2);
    return(P_meanDes*1e-5);

   /* t1dot = R*theta_dot/(lo*emax);
    t2dot = (I*theta_dot3 + fv*theta_dot2 + m*g*link_l*0.5*cos(theta))/(R*Fmax);

    P_real_dot = Pmax*(t1dot + t2dot);
    
    t1dot2 = R*theta_dot2/(lo*emax);
    t2dot2 = (I*theta_dot4 + fv*theta_dot3 - m*g*link_l*0.5*sin(theta))/(R*Fmax);
    P_real_dot2 = Pmax*(t1dot2 + t2dot2);

    P = [P1 P2];
    Pdot = [P1dot P2dot];
    Pdot2 = [P1dot2 P2dot2];*/
}


int main()
{
   
    struct timeval tbegin,tend;
    double texec=0.0;
    stateVec_t xinit,xDes, refDes;
    //VectorXd refDes;
    double Pfeed;
    PneumaticarmModel model;
    xinit << 0.0,   0.0, 0.0,   4.0*1e5;
    //xDes << 1.0,    0.0, 2.0*1e5, 2.0*1e5;

    unsigned int T = 10;
    unsigned int M = 800;
    unsigned int finiter = (unsigned int) M/T;
    unsigned int lp = 0;
    double dt=5e-3;
    unsigned int iterMax = 20;
    double stopCrit = 1e-5;
    stateVec_t* xList;
    commandVec_t* uList;
    ILQRSolver::traj lastTraj;

    //RomeoSimpleActuator romeoActuatorModel(dt);
    //RomeoLinearActuator romeoLinearModel(dt);
    PneumaticarmNonlinearModel pneumaticarmModel(dt);
    //PneumaticarmElbowPieceLinear pneumaticPieceLinearModel(dt);
    //CostFunctionRomeoActuator costRomeoActuator;
    CostFunctionPneumaticarmElbow costPneumatic;
    ILQRSolver testSolver(pneumaticarmModel,costPneumatic);
    

    ofstream fichier("/home/gkharish/softdev/DDP/matlab/resultsMPC.csv",ios::out | ios::trunc);
    if(!fichier)
    {
        cerr << "erreur fichier ! " << endl;
        return 1;
    }
    fichier << T << "," << M << endl;
    fichier << "theta,ref, thetaDot,P1,P2,u1,u2" << endl;


    //testSolverRomeoActuator.FirstInitSolver(xinit,xDes,T,dt,iterMax,stopCrit);
    // Xdes Reference trajectory


    gettimeofday(&tbegin,NULL);
    model.setParameters();
    for(unsigned int ix =0;ix<4;ix++)
        model.Set_StateVector(xinit(ix),ix);

    for(int i=0;i<M;i++)
    {
       refDes(0) = dt*i*10*3.14/180;
       refDes(1) = 10*3.14/180;
       refDes(2) = 0;
       Pfeed = InverseModel(refDes);
       //cout << "Pfeed: " << Pfeed;
       xDes(0) = dt*i*10*3.14/180;
       xDes(1) = 10*3.14/180;
       xDes(2) = Pfeed*1e5;
       xDes(3) = 4*1e5 - Pfeed*1e5;
       for(unsigned int ix =0;ix<4;ix++)
           xinit(ix) = model.Get_StateVector(ix);

       testSolver.FirstInitSolver(xinit,xDes,T,dt,iterMax,stopCrit);
       testSolver.initSolver(xinit,xDes);
       testSolver.solveTrajectory();
       lastTraj = testSolver.getLastSolvedTrajectory();
       xList = lastTraj.xList;
       uList = lastTraj.uList;
       for(unsigned int ic =0;ic<2;ic++) 
           model.Set_ControlVector(uList[0](ic,0),ic);
       double t = i*dt;
       model.integrateRK4(t,dt);
       //xinit = xList[1];
       /* lp = lp+1;
        if(lp >= finiter)
        {
            lp =0;
            T=T-1;

        }*/
        /*for(int j=0;j<T;j++) fichier << xList[j](0,0) << "," << xList[j](1,0) << "," << xList[j](2,0)  << "," << uList[j](0,0) << endl;*/
        fichier << xList[1](0,0) << "," << refDes(0) << "," << xList[1](1,0) << "," << xList[1](2,0) << "," << xList[1](3,0)<< "," << uList[1](0,0) << "," << uList[1](1,0) << endl;
    }
    gettimeofday(&tend,NULL);


    texec=((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;
    texec = (double)(tend.tv_usec - tbegin.tv_usec);

    cout << "temps d'execution total du solveur ";
    cout << texec/1000000.0 << endl;
    cout << "temps d'execution par pas de MPC ";
    cout << texec/(T*1000000) << endl;

//    fichier.close();


    return 0;

}

