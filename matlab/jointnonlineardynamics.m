% function [jointstate_deriv,F, T, Jx, Ju] = jointnonlineardynamics(joint_state, dt,u)
% jointstate_deriv = zeros(4,1);
% theta = joint_state(1);
% theta_dot = joint_state(2);
% P1 = joint_state(3);
% P2 = joint_state(4);
% %% Parameters for the muscles
% lo = 0.185;
% alphao = 23.0*pi/180;
% %epsilono = 0.15;
% %emax = 0.2983;
% k = 1.25;
% ro = 0.0085;
% R = 0.015;
% 
% a = 3/(tan(alphao))^2;
% b = 1/(sin(alphao))^2;
% emax = (1/k)*(1 - sqrt(b/a));
% 
% lb = lo- R*theta;
% epsb = (1-(lb/lo));
% lt = lo*(1-emax) + R*theta;
% epst = (1-(lt/lo));
% 
% %% Parameters of Joint
% m = 2.6;
% link_l = 0.32;
% g =9.81;
% I = m*(link_l^2)/3;
% fv = 0.25;
% 
% 
% % F1 =  pi*ro^2*P1*1e5*(a*(1-k*epsb)^2 - b);
% % F2 =  pi*ro^2*P2*1e5*(a*(1-k*epst)^2 - b);
% % F2max = 1*pi*ro^2*4*1e5*(a*(1-k*emax)^2 - b);
% % T = (F1 -F2 )*R;
% % F = [F1 F2 T];
% 
% 
% %% Non linear modelling
% 
% co = pi*ro^2*1e5;
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
% 
% % jointstate_deriv(1) = theta_dot; %joint_state(2);
% % jointstate_deriv(2) = ((F1 -F2 )*R  - fv*theta_dot - m*g*0.5*link_l*sin(theta))/I;
% 
% %% Calculation of Jacobian
% Tc = 0.17;
% Jx = zeros(4,4);
% Jx(1,:) = [0 1 0 0];
% Jx(3,:) = [0 0 (-1/Tc) 0];
% Jx(4,:) = [0 0 0 (-1/Tc)];
% 
% %% J(2,1)
% 
% t1_j21  = - m*g*0.5*link_l*cos(theta)/I;
% t2_j21 = 2*co*a*k^2*emax*(R/lo)*P2;
% t3_j21 = 2*co*a*((k*R/lo)^2)*(P1 - P2)*theta;
% t4_j21 = -2*co*a*k*(R/lo)*(P1 + P2);
% Jx(2,1) = t1_j21 + (t2_j21 + t3_j21 + t4_j21)*(R/I);
% %% J(2,2)
% Jx(2,2) = -fv;
% %% J(2,3)
% 
% t1_j23 = co*(a-b);
% t2_j23 = -2*co*a*k*(R/lo)*theta;
% t3_j23 = co*a*((k*R/lo)^2)*theta^2;
% 
% Jx(2,3) = (t1_j23  + t2_j23 + t3_j23 )*(R/I);
% %% j(2,4)
% 
% t1_j24 =co*(a-b);
% 
% t2_j24 = co*a*((k*emax)^2 - 2*k*emax);
% tt3_1 = (R*theta/lo)^2 - 2*emax*(R*theta/lo);
% 
% t3_j24 = co*a*(k^2*tt3_1 + 2*k*(R*theta/lo));
% 
% Jx(2,4) = -(t1_j24 + t2_j24 + t3_j24)*(R/I);
% 
% Ju = zeros(4,2);
% Ju(:,1) = [0 0 1/Tc 0]';
% Ju(:,1) = [0 0 0 1/Tc]';
% Ad = Jx*dt + eye(4);
% Bd = Ju*dt;
% 
% %% Jxx
% Jxx = zeros(4,4);
% jxx(2,1) = (m*g*0.5*link_l*sin(theta)/I) + 2*co*a*((k*R/lo)^2)*(P1 - P2)*(R/I);
% jointstate_deriv = Ad*joint_state + Bd*u;
% 
% 
% %%
%% Parameters for the muscles


function [jointstate_deriv,F, T, Jx, Ju] = jointnonlineardynamics(joint_state, dt,Uc)
%jointstate_deriv = zeros(4,1);
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
eo = 0;

lb = lo*(1-eo)- R*theta;
epsb = (1-(lb/lo));
lt = lo*(1-emax) + R*theta;
epst = (1-(lt/lo));

%% Parameters of Joint
m = 2.6;
link_l = 0.32;
g =9.81;
I = m*(link_l^2)/3;
fv = 0.25;


% F1 =  pi*ro^2*P1*1e5*(a*(1-k*epsb)^2 - b);
% F2 =  pi*ro^2*P2*1e5*(a*(1-k*epst)^2 - b);
% F2max = 1*pi*ro^2*4*1e5*(a*(1-k*emax)^2 - b);
% T = (F1 -F2 )*R;
% F = [F1 F2 T];


%% Non linear modelling

co = pi*ro^2;
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

% jointstate_deriv(1) = theta_dot; %joint_state(2);
% jointstate_deriv(2) = ((F1 -F2 )*R  - fv*theta_dot - m*g*0.5*link_l*sin(theta))/I;

%% Calculation of Jacobian
% Tc = 0.17;
% Jx = zeros(4,4);
% Jx(1,:) = [0 1 0 0];
% Jx(3,:) = [0 0 (-1/Tc) 0];
% Jx(4,:) = [0 0 0 (-1/Tc)];
% 
% %% J(2,1)
% 
% t1_j21  = -(m*g*0.5*link_l*cos(theta))/I;
% t2_j21 = 2*co*a*k^2*emax*(R/lo)*P2;
% t3_j21 = 2*co*a*((k*R/lo)^2)*(P1 - P2)*theta;
% t4_j21 = -2*co*a*k*(R/lo)*(P1 + P2);
% Jx(2,1) = t1_j21 + (t2_j21 + t3_j21 + t4_j21)*(R/I);
% %% J(2,2)
% Jx(2,2) = -fv/I;
% %% J(2,3)
% 
% t1_j23 = co*(a-b);
% t2_j23 = -2*co*a*k*(R/lo)*theta;
% t3_j23 = co*a*((k*R/lo)^2)*theta^2;
% 
% Jx(2,3) = (t1_j23  + t2_j23 + t3_j23 )*(R/I);
% %% j(2,4)
% 
% t1_j24 =co*(a-b);
% 
% t2_j24 = co*a*((k*emax)^2 - 2*k*emax);
% tt3_1 = (R*theta/lo)^2 - 2*emax*(R*theta/lo);
% 
% t3_j24 = co*a*(k^2*tt3_1 + 2*k*(R*theta/lo));
% 
% Jx(2,4) = -(t1_j24 + t2_j24 + t3_j24)*(R/I);

%% Alternative calculation of jacobian
% Tc1 = 0.18;
% Tc2 = 0.13;
% Jx = zeros(4,4);
% Jx(1,:) = [0 1 0 0];
% Jx(3,:) = [0 0 (-1/Tc1) 0];
% Jx(4,:) = [0 0 0 (-1/Tc2)];
% t1_j21  = - m*g*0.5*link_l*cos(theta)/I;
%  t1 = -pi*ro^2*P1*(2*a*k*(1-k*epsb)*(R/lo));
%  t2 = -pi*ro^2*P2*(2*a*k*(1-k*epst)*(R/lo));
%  Jx(2,1) =  t1_j21 + (t1 + t2)*(R/I);
%  Jx(2,2) = -fv/I;
%  Jx(2,3) = (R/I)*pi*ro^2*(a*(1-k*epsb)^2 - b);
%  Jx(2,4) = (-R/I)*pi*ro^2*(a*(1-k*epst)^2 - b);
% F2 =  pi*ro^2*P2*1e5*(a*(1-k*epst)^2 - b);

%% Numerical differentiation for jacobian calculation

dx1 = 1e-4;
dx2 = 1e-4;
dx3 = 1e1;
dx4 = 1e1;
Tc1 =0.18;
Tc2 = 0.13;
Jx(1,:) = [0 1 0 0];
Jx(3,:) = [0 0 (-1/Tc1) 0];
Jx(4,:) = [0 0 0 (-1/Tc2)];
%% 
%Jx(2,1)
X=joint_state;
fxdx2 = joint_state(1) + dx1;
fxdx1 = joint_state(1) - dx1;
X(1) = fxdx2;
f2_theta2 = dtheta_dot(X)
X(1) = fxdx1;
f2_theta1 = dtheta_dot(X);
Jx(2,1) = (f2_theta2 - f2_theta1)/(2*dx1);
% Jx(2,2)
Jx(2,2) = -fv/I;
%Jx(2,3)
X=joint_state;
fxdx2 = joint_state(3) + dx3;
fxdx1 = joint_state(3) - dx3;
X(3) = fxdx2;
f2_theta2 = dtheta_dot(X);
X(3) =  fxdx1;
f2_theta1 = dtheta_dot(X)
Jx(2,3) = (f2_theta2 - f2_theta1)/(2*dx3);

%Jx(2,4)
X=joint_state;
fxdx2 = joint_state(4) + dx4;
fxdx1 = joint_state(4) - dx4;
X(4) = fxdx2;
f2_theta2 = dtheta_dot(X);
X(4) = fxdx1;
f2_theta1 = dtheta_dot(X);
Jx(2,4) = (f2_theta2 - f2_theta1)/(2*dx4);

%%
Ju = zeros(4,2);
Ju(:,1) = [0 0 1/Tc1 0]';
Ju(:,2) = [0 0 0 1/Tc2]';
Ad = Jx*dt + eye(4);
Bd = Ju*dt;


jointstate_deriv = Ad*joint_state + Bd*Uc;
F = 0;
T =0;
end

% function func1 = dtheta(joint_state)
% func1 = joint_state(2);
% end

function func2 = dtheta_dot(joint_state)

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
func2 =  ((F1 -F2 )*R  - fv*theta_dot - m*g*0.5*link_l*sin(theta))/I;
end

% function func3 = dP1(joint_state, u)
% theta = joint_state(1);
% theta_dot = joint_state(2);
% P1 = joint_state(3);
% P2 = joint_state(4);
% Tc1 = 0.18;
% func3 = -P1/Tc1 + u*1e5/Tc1;
% end
% 
% function func4 = dP2(joint_state, u)
% theta = joint_state(1);
% theta_dot = joint_state(2);
% P1 = joint_state(3);
% P2 = joint_state(4);
% Tc2 = 0.13;
% func4 = -P2/Tc2 + u*1e5/Tc2;
% end