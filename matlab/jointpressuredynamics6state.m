function [jointstate_deriv,F,V,Pstate_deriv] = jointpressuredynamics6state(joint_state, Pdes, P_state)
jointstate_deriv = zeros(2,1);
F = zeros(2,1);
V = zeros(2,1);
Pstate_deriv = zeros(4,1);
theta = joint_state(1);
theta_dot = joint_state(2);
%% Parameters for the muscles
lo = 0.185;
alphaob = 23.0*pi/180;
alphaot = 23.0*pi/180;
k = 1.25;
ro = 0.009;
R = 0.015;

a_biceps = 3/(tan(alphaob))^2;
b_biceps = 1/(sin(alphaob))^2;
emax_biceps = (1/k)*(1 - sqrt(b_biceps/a_biceps));

a_triceps = 3/(tan(alphaot))^2;
b_triceps = 1/(sin(alphaot))^2;
emax_triceps = (1/k)*(1 - sqrt(b_triceps/a_triceps));

lb = lo- R*theta;
epsb = (1-(lb/lo));
lt = lo*(1-emax_triceps) + R*theta;
epst = (1-(lt/lo));

%% Parameters of Joint
m = 2.6;
link_l = 0.32;
g =9.81;
I = m*(link_l^2)/3;
fv = 0.25;
%% Volume calcuation
% biceps agonistic muscle
lb = lo - R*theta;
csb2 = (cos(alphaob))^2;
epsb2 = (1-(lb/lo))^2;
termb1 = (1 - csb2*epsb2);

Vb = 1e6*(pi*lb*ro^2/((sin(alphaob))^2))*termb1
%Vb = 1630;
wnb = 2*pi*380*(1/Vb)

% triceps antagonistic muscle 
lt = lo*(1-emax_triceps) + R*theta; 
cst2 = (cos(alphaot))^2;
epst2 = (1-(lt/lo))^2;
termt1 = (1 - cst2*epst2);

Vt = 1e6*(pi*lt*ro^2/((sin(alphaot))^2))*termt1
%Vt = 1630;
wnt = 2*pi*380*(1/Vt)

V = [Vb;Vt];

%% Pressure Dynamics
Pb_state = P_state(1:2,1);
Pt_state = P_state(3:4,1);
Pstate_deriv(1) = Pb_state(2);
Pstate_deriv(2) = -wnb^2*Pb_state(1) - 2*wnb*1*Pb_state(2) + (wnb^2)*Pdes(1);

Pstate_deriv(3) = Pt_state(2);
Pstate_deriv(4) = -wnt^2*Pt_state(1) - 2*wnt*1*Pt_state(2) + (wnt^2)*Pdes(2);

%% Force calculation
P1 = Pb_state(1);
P2 = Pt_state(1);
F_biceps =  pi*ro^2*P1*1e5*(a_biceps*(1-k*epsb)^2 - b_biceps);
F_triceps =  pi*ro^2*P2*1e5*(a_triceps*(1-k*epst)^2 - b_triceps);
%F2max = 1*pi*ro^2*4*1e5*(a*(1-k*emax)^2 - b);
F = [F_biceps; F_triceps];

%% Joint Dynamics
jointstate_deriv(1) = theta_dot; %joint_state(2);
jointstate_deriv(2) = ((F_biceps -F_triceps )*R  - fv*theta_dot - m*g*0.5*link_l*sin(theta))/I;


%% END