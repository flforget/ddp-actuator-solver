function [jointstate_deriv,F, T] = jointdynamics4state(u1,u2,joint_state)
jointstate_deriv = zeros(4,1);
theta = joint_state(1);
theta_dot = joint_state(2);
P1 = joint_state(3);
P2 = joint_state(4);
%% Parameters for the muscles
lo = 0.185;
alphao = 23.0*pi/180;
%epsilono = 0.15;
%emax = 0.2983;
k = 1.25;
ro = 0.0085;
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


F1 =  pi*ro^2*P1*(a*(1-k*epsb)^2 - b);
F2 =  pi*ro^2*P2*(a*(1-k*epst)^2 - b);
F2max = 1*pi*ro^2*4*1e5*(a*(1-k*emax)^2 - b);
T = (F1 -F2 )*R;
F = [F1 F2 T];


%% Non linear modelling
Tc2 =0.13; 
Tc1 = 0.19;
% co = pi*ro^2;
% tb1 = co*(a -b)*P1;
% tb2 = co*a*(-2*k)*(R/lo)*P1*theta;
% tb3 = co*a*(k^2*R^2/(lo^2))*P1*(theta^2);
% F1 = tb1 + tb2 + tb3;
% 
% tt1 = co*(a -b)*P2;
% 
% tt2_1 = (k*emax)^2 - 2*k*emax;
% tt2 = co*a*tt2_1*P2;
% 
% tt3_1 = (R*theta/lo)^2 - 2*emax*(R*theta/lo);
% 
% tt3 = co*a*(k^2*tt3_1 + 2*k*(R*theta/lo))*P2;
% 
% F2 = tt1 +tt2 +tt3;
% 
% T = (F1 -F2 )*R;
% F = [F1 F2 T];

jointstate_deriv(1) = theta_dot; %joint_state(2);
jointstate_deriv(2) = ((F1 -F2 )*R  - fv*theta_dot - m*g*0.5*link_l*sin(theta))/I;
jointstate_deriv(3) = -P1/Tc1 + u1*1e5/Tc1;
jointstate_deriv(4) = -P2/Tc2 + u2*1e5/Tc2;
