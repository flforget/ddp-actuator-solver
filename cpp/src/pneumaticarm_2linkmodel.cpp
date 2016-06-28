// Authors: Ganesh Kumar
/* ******** It is a trial program for modelling a Pneumatic muscle and
   ******** controlling it in closeloop under xenomai realtime kernel ********/



#include "pneumaticarm_2linkmodel.hh"
#include <math.h>
using namespace std;
//#define pi M_PI
double PI = 3.14;
// Joint 1 parameters
// Joint 2 parameters
double j2lo1 = 0.23;
double j2alphao1 = 20*PI/180;
double j2k1 = 1.1;
double j2ro1 = 0.012;
double j2R1 = 0.009;
double j2m1 = 2.7;
double j2link1_l = 351.1*1e-3;
double j2link1_lc = 125.4*1e-3;
double j2link1_I = 0.02;
double j2fv1 = 3.0;
double j2Pmax1 = 3.0;
// Joint 3 parameters
double j4lo = 0.185;
double j4alphao = 23*PI/180;
double j4k = 1.25;
double j4ro = 0.0085;
double j4R = 0.015;
double j4m2 = 2.6;
double j4link2_l = 307e-3;
double j4link2_lc = 178e-3;
double j4link2_I = 0.0144;
double j4fv2 = 0.25;
double j4Pmax = 4;
double pi =3.14;
double mb = 0.01; //mass of the load

double Torque_net(stateVec_t  x,double lo,double alphaob,double k,
                                            double ro,double R,unsigned int i,double pmax)
{
double theta = x(i);
double a_biceps = 3/pow(tan(alphaob),2);
double b_biceps = 1/pow(sin(alphaob),2);
double emax_biceps = (1/k)*(1 - sqrt(b_biceps/a_biceps));
double alphaot = alphaob;
double a_triceps = 3/pow(tan(alphaot),2);
double b_triceps = 1/pow(sin(alphaot),2);
double emax_triceps = (1/k)*(1 - sqrt(b_triceps/a_triceps));

double lb = lo - R*theta;
double epsb = (1-(lb/lo));
double lt = lo*(1-emax_triceps) + R*theta;
double epst = (1-(lt/lo));

double P1 = x(i+4);
double P2 = pmax-x(i+4);
//P = [P1;P2];
double fbterm = 1e5*pi*pow(ro,2)*(a_biceps*pow((1-k*epsb),2) - b_biceps);
double F_biceps =  P1*fbterm;
double ftterm = 1e5*pi*pow(ro,2)*(a_triceps*pow((1-k*epst),2) - b_triceps);
double F_triceps = P2*ftterm;
//%F2max = 1*pi*ro^2*4*1e5*(a*pow((1-k*emax),2) - b);
//Fmat = [F_biceps; F_triceps];
double Torque_pneumatics = (F_biceps -F_triceps )*R;
return (Torque_pneumatics);
}
stateVec_t computejointderiv(double& dt, const stateVec_t& Xe,const stateVec_t& Xdes,const commandVec_t& U)
{
    //    result(1,0)-=A10*sin(X(0));
    //result(3,0)+=A33atan*atan(a*X(3,0));
    stateVec_t X;
    X = Xe;// + Xdes;
    stateVec_t jointstate_deriv;
    double m = -0.0023;
    double c = 0.0136;
    double p1 = -0.009338;   //%(-0.01208, -0.006597)
    double p2 = 0.01444;

    j2R1 = p1*X(0) + p2;
    //P1_ = X(4);
    //P2_ = X(5);
    double wnb1 = 9;
    double wnb2 = 8;
    //%% Delta P Pressure Dynamics
    ////%%%%%%% 2nd order  %%%%%%%%%%%%%%
    ////%wnb2 = wnb1;a
    double Pdes1 = U(0)+1.5;
    double Pdes2 = U(1)+2.0;
    jointstate_deriv(4) = X(6);
    jointstate_deriv(5) = X(7);
    jointstate_deriv(6) = -pow(wnb1,2)*X(4) - 2*wnb1*X(6) + pow(wnb1,2)*Pdes1;
    jointstate_deriv(7) = -pow(wnb2,2)*X(5) - 2*wnb2*X(7) + pow(wnb2,2)*Pdes2;

    //%% Force calculation
    double T1 = Torque_net(X,j2lo1,j2alphao1,j2k1,j2ro1,j2R1,0,j2Pmax1);
    double T2 = Torque_net(X,j4lo,j4alphao,j4k,j4ro,j4R,1,j4Pmax);

    //%% Mass Inertia Matrix
    double m11_const = j2link1_I + j2m1*pow(j2link1_lc,2) + j4link2_I + j4m2*(pow(j2link1_l,2) +
                                pow(j4link2_lc,2)) + mb*(pow(j2link1_l,2) + pow(j4link2_l,2));

    double m11_var = j4m2*2*j2link1_l*j4link2_lc*cos(X(1)) +
                                mb*2*j2link1_l*j4link2_l*cos(X(1));

    double m11 = (m11_var + m11_const);
    double m12_const = j4link2_I + j4m2*pow(j4link2_lc,2) + mb*pow(j4link2_l,2);
    double m12_var = j4m2*j2link1_l*j4link2_lc*cos(X(1)) +
                                mb*j2link1_l*j4link2_l*cos(X(1));

    double m12 = (m12_var + m12_const);
    double m22 = j4link2_I + j4m2*pow(j4link2_lc,2) + mb*pow(j4link2_l,2);
    double det = m11*m22 - m12*m12;
    double inv_m11 = m22/det;
    double inv_m22 = m11/det;
    double inv_m12 = -m12/det;
    double inv_m21 = -m12/det;
    //%% Coriolis Matrix
    double c1_const = -(j4m2*j4link2_lc + mb*j4link2_l)*j2link1_l;
    double c1_var1 = sin(X(1));
    double c1_var2 = 2*X(2)*X(3) + pow(X(3),2);
    double c1 = c1_const*(c1_var1*c1_var2);
    double c2_const = (j4m2*j4link2_lc + mb*j4link2_l)*j2link1_l;
    double c2_var1 = sin(X(1));
    double c2_var2 = pow(X(2),2);
    double c2 = c2_const*(c2_var1*c2_var2);

    //%% Gravity Matrix
    double g1 = (j2m1*j2link1_lc + j4m2*j2link1_l + mb*j2link1_l)*sin(X(0)) + (
                        j4m2*j4link2_lc + mb*j4link2_l)*sin(X(0) + X(1));

    double g2 = 9.8*(j4m2*j4link2_lc + mb*j4link2_l)*sin(X(0) + X(1));

    //%% viscous friction matrix
    double tf1 = -j2fv1*X(2);
    double tf2 = -j4fv2*X(3);

    double Mat1 = inv_m11*(T1 + tf1 - c1 - 9.8*g1) + inv_m12*(T2+tf2 -c2 -g2);
    double Mat2 = inv_m21*(T1 + tf1 - c1 - 9.8*g1) + inv_m22*(T2+tf2 -c2 -g2);

    jointstate_deriv(0) = X(2); //%joint_state(2);
    jointstate_deriv(1) = X(3); //%joint_state(2);
    jointstate_deriv(2) = Mat1;
    jointstate_deriv(3) = Mat2;
    stateVec_t result = X + dt*jointstate_deriv;

    //fx = jointstate_deriv;

    return result;
}



PneumaticarmNonlinearModel::PneumaticarmNonlinearModel(double& mydt)
{
    stateNb=8;
    commandNb=2;
    x1.resize(2);
    dt = mydt;
    //state_vector_.resize(8);
    //state_derivative_.resize(8);
    //control_vector_.resize(2);
   //////////////////////////////////////////////
    /*fxx[0].setZero();
    fxx[1].setZero();
    fxx[2].setZero();
    fxx[3].setZero();

    fxu[0].setZero();
    fxu[1].setZero();

    fuBase = B;
    fu = dt* fuBase;
    fuu[0].setZero();
    fux[0].setZero();
    fxu[0].setZero();

    QxxCont.setZero();
    QuuCont.setZero();
    QuxCont.setZero();*/

    lowerCommandBounds << -1.5,-2.0;
    upperCommandBounds << 1.5,2.0;
    commandOffset << 1.5,2.0;
}

stateVec_t PneumaticarmNonlinearModel::computeNextState(double& dt, const stateVec_t& Xe,const stateVec_t& xDes,const commandVec_t& U)
{
    //    result(1,0)-=A10*sin(X(0));
    //result(3,0)+=A33atan*atan(a*X(3,0));
    stateVec_t jointstate_deriv;
    stateVec_t X = Xe + xDes;
    double m = -0.0023;
    double c = 0.0136;
    double p1 = -0.009338;   //%(-0.01208, -0.006597)
    double p2 = 0.01444;

    j2R1 = p1*X(0) + p2;

    //double P1_ = X(4);
    //double P2_ = X(5);
    double wnb1 = 9;
    double wnb2 = 8;
    //%% Delta P Pressure Dynamics
    ////%%%%%%% 2nd order  %%%%%%%%%%%%%%
    ////%wnb2 = wnb1;a
    double Pdes1 = U(0)+commandOffset(0);
    double Pdes2 = U(1)+commandOffset(1);
    jointstate_deriv(4) = X(6);
    jointstate_deriv(5) = X(7);
    jointstate_deriv(6) = -pow(wnb1,2)*X(4) - 2*wnb1*X(6) + pow(wnb1,2)*Pdes1;
    jointstate_deriv(7) = -pow(wnb2,2)*X(5) - 2*wnb2*X(7) + pow(wnb2,2)*Pdes2;

    //%% Force calculation
    double T1 = Torque_net(X,j2lo1,j2alphao1,j2k1,j2ro1,j2R1,0,j2Pmax1);
    double T2 = Torque_net(X,j4lo,j4alphao,j4k,j4ro,j4R,1,j4Pmax);

    //%% Mass Inertia Matrix
    double m11_const = j2link1_I + j2m1*pow(j2link1_lc,2) + j4link2_I + j4m2*(pow(j2link1_l,2) +
                                pow(j4link2_lc,2)) + mb*(pow(j2link1_l,2) + pow(j4link2_l,2));

    double m11_var = j4m2*2*j2link1_l*j4link2_lc*cos(X(1)) +
                                mb*2*j2link1_l*j4link2_l*cos(X(1));

    double m11 = (m11_var + m11_const);

    double m12_const = j4link2_I + j4m2*pow(j4link2_lc,2) + mb*pow(j4link2_l,2);
    double m12_var = j4m2*j2link1_l*j4link2_lc*cos(X(1)) +
                                mb*j2link1_l*j4link2_l*cos(X(1));

    double m12 = (m12_var + m12_const);
    double m22 = j4link2_I + j4m2*pow(j4link2_lc,2) + mb*pow(j4link2_l,2);
    double det = m11*m22 - m12*m12;
    double inv_m11 = m22/det;
    double inv_m22 = m11/det;
    double inv_m12 = -m12/det;
    double inv_m21 = -m12/det;
    //%% Coriolis Matrix
    double c1_const = -(j4m2*j4link2_lc + mb*j4link2_l)*j2link1_l;
    double c1_var1 = sin(X(1));
    double c1_var2 = 2*X(2)*X(3) + pow(X(3),2);
    double c1 = c1_const*(c1_var1*c1_var2);
    double c2_const = (j4m2*j4link2_lc + mb*j4link2_l)*j2link1_l;
    double c2_var1 = sin(X(1));
    double c2_var2 = pow(X(2),2);
    double c2 = c2_const*(c2_var1*c2_var2);

    //%% Gravity Matrix
    double g1 = (j2m1*j2link1_lc + j4m2*j2link1_l + mb*j2link1_l)*sin(X(0)) + (
                        j4m2*j4link2_lc + mb*j4link2_l)*sin(X(0) + X(1));

    double g2 = 9.8*(j4m2*j4link2_lc + mb*j4link2_l)*sin(X(0) + X(1));

    //%% viscous friction matrix
    double tf1 = -j2fv1*X(2);
    double tf2 = -j4fv2*X(3);

    double Mat1 = inv_m11*(T1 + tf1 - c1 - 9.8*g1) + inv_m12*(T2+tf2 -c2 -g2);
    double Mat2 = inv_m21*(T1 + tf1 - c1 - 9.8*g1) + inv_m22*(T2+tf2 -c2 -g2);

    jointstate_deriv(0) = X(2); //%joint_state(2);
    jointstate_deriv(1) = X(3); //%joint_state(2);
    jointstate_deriv(2) = Mat1;
    jointstate_deriv(3) = Mat2;
    stateVec_t result = X + dt*jointstate_deriv ;
    x1[0] = X(0); x1[1] = X(1);
    //fx = jointstate_deriv;
    return result;
}

void PneumaticarmNonlinearModel::computeAllModelDeriv(double& dt, const stateVec_t& Xe,const stateVec_t& Xdes,const commandVec_t& U)
{
    //fx = fxBase;
    commandVec_t Ureal;
    Ureal(0) = U(0) +commandOffset(0);
    Ureal(1) = U(1) +commandOffset(1);
    Xreal = Xe + Xdes;
    stateVec_t X = Xe + Xdes;
    double dh = 1e-5;
    stateVec_t tempX, derivplus, derivminus;
    commandVec_t tempU;

    for(unsigned int i = 0; i<8; i++)
    {
        for(unsigned int j=0; j<8;j++)
        {
            tempX = X;
            tempX(j) = X(j) + dh;
            derivplus = computejointderiv(dt, tempX,Xdes, Ureal); //cout<< "deriv:" <<derivplus(0)<< '\n' << endl;
            tempX = X;
            tempX(j) = X(j) - dh;
            derivminus = computejointderiv(dt, tempX,Xdes, Ureal);
            fx(i,j) = (derivplus(i) - derivminus(i))/(2*dh);
        }
    }
    derivplus.setZero(); derivminus.setZero();
    for(unsigned int i = 0; i<8; i++)
    {
        for(unsigned int j=0; j<2;j++)
        {
            tempU = Ureal;
            tempU(j) = Ureal(j) + dh;
            derivplus = computejointderiv(dt, X,Xdes, tempU);
            tempU = Ureal;
            tempU(j) = Ureal(j) - dh;
            derivminus = computejointderiv(dt, X,Xdes, tempU);
            fu(i,j) = (derivplus(i) - derivminus(i))/(2*dh);
        }
    }
   //fx.setZero(); fu.setZero();
}

stateMat_t PneumaticarmNonlinearModel::computeTensorContxx(const stateVec_t& nextVx)
{
    QxxCont.setZero();//nextVx[0]*fxx[0] + nextVx[1]*fxx[1] + nextVx[2]*fxx[2] + nextVx[3]*fxx[3];
    return QxxCont;
}

commandMat_t PneumaticarmNonlinearModel::computeTensorContuu(const stateVec_t& nextVx)
{
    return QuuCont;
}

commandR_stateC_t PneumaticarmNonlinearModel::computeTensorContux(const stateVec_t& nextVx)
{
    return QuxCont;
}

/// accessors ///
/*stateMat_t& PneumaticarmNonlinearModel::getfx()
{
    return fx;
}

stateR_commandC_t& PneumaticarmNonlinearModel::getfu()
{
    return fu;
}

double PneumaticarmNonlinearModel::getX(unsigned int i)
{
    return x1[i];
}
*/
