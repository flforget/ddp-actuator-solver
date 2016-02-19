function [xtrajout, uhat, Tnet, s] = demo_pneumatic2link_delP_DDP
% A demo of iLQG/DDP with pneumatic 2link manipulator-dynamics

fprintf(['\nA demonstration of the iLQG algorithm '...
'with car parking dynamics.\n'...
'for details see\nTassa, Mansard & Todorov, ICRA 2014\n'...
'\"Control-Limited Differential Dynamic Programming\"\n'])

% Set full_DDP=true to compute 2nd order derivatives of the 
% dynamics. This will make iterations more expensive, but 
% final convergence will be much faster (quadratic)
full_DDP = false;

dt = 5e-3; % dt for dynamics
ToT = 1;       %(in seconds)
N_tot = ToT/dt;
%T_horizon = 0.5;  %(in seconds)
T       = N_tot; %10; %T_horizon/dt;
%N_DT = ToT/T_horizon;


x0      = zeros(8,1); %[0.0;0;0.0e5;0e5];   % initial state

u0      = 0.1*randn(2,T);
xtot = zeros(8,ToT/dt);
final_goal = [0.5 1 0 0 0*1.238e5 0*1.578e5 0 0]';

xGoal = zeros(8,1);
xGoal = [0.5 1 0 0 1.238 1.578 0 0]';
%% MPC
for k=1:1
    xGoal(1,1) = 0.5; %a1*sin(f1*k*dt); 
    xGoal(3,1) = 0; %f1*a1*cos(f1*k*dt); %slope_pos1;
    xGoal(5,1) = 0;  %slope_pres1*k*dt;
    xGoal(7,1) = 0;  %slope_pres1;
    %testgoal1(k) = a1*sin(f1*k*dt);
     % Trajectory for Joint 2
    xGoal(2,1) = 1 %a2*sin(f2*k*dt); 
    xGoal(4,1) = 0 %f2*a2*cos(f2*k*dt);  %slope_pos2;
    xGoal(6,1) = 0;  %slope_pres2*k*dt;
    xGoal(8,1) = 0;  %slope_pres2;
    %testgoal2(k) = a2*sin(f2*k*dt); 
    xGoal(9,1) =  0; %-f1*f1*a1*sin(f1*k*dt); %0;
    xGoal(10,1) =  0; %-f2*f2*a2*sin(f2*k*dt);%0;
    xGoal(11,1) = 0; %-f1*f1*f1*a1*cos(f1*k*dt);
    xGoal(12,1) = 0; %-f2*f2*f2*a2*cos(f2*k*dt);
% optimization problem
DYNCST  = @(x,u,i) pneumatic_dyn_cst(x,u,full_DDP, xGoal);
Op.lims  = [0.0e5 3.0; 0.0e5 3.5];
Op.maxIter = 250;
% run the optimization
Op.plot = 0;    
[xhat,uhat, Lhat]= iLQG(DYNCST, x0, u0, Op);
[row,col] = size(uhat)
for i=1:col
xhat0 = xhat(1:8,i);
uhat0 = uhat(1:2,i);
xtraj(1:8,i) = pneumatics6st_dynamics(xhat0,uhat0);
%% Stiffness calculation
dx = [1e-2;1e-2];
Tnet(1:2,i) = Net_Torue(xhat0,uhat0);
xtemp(1:8,i) = xhat0;
xtemp(1:2,i) = xhat0(1:2,1)+dx; 
f2(1:2,i) = Net_Torue(xtemp,uhat0);

xtemp(1:8,i) = xhat0;
xtemp(1:2,i) = xhat0(1:2,1)-dx; 
f1(1:2,i) = Net_Torue(xtemp,uhat0);
s= -(f2 -f1)./(2*dx(1));
end
%st = finite_difference(fun,xu,h);

% size(x(1:4,1:T))
% size(xtot(1:4,xinit:xfin))
% xtot(1:4,xinit:xfin) = x(1:4,1:T);
% utot(1,T*(k-1)+1:T*k) = u(1,1:T);
end
%print('END OF LOOP')

xtrajout = xhat(1:8,1:N_tot);
%% PLOT

%figure(4)
% subplot(221), plot(xhat(1,:));
% subplot(222), plot(uhat(1,:));
% hold on;
% subplot(222), plot(u(2,:),'g');
% 
% subplot(223), plot(x(3,:));
% subplot(224), plot(x(4,:));
figure(1)
subplot(211), plot((1:N_tot)*dt, (180/pi)*xhat(1,1:N_tot), 'b', 'Linewidth', 2.0);
hold on;
grid on;
subplot(211), plot((1:N_tot)*dt, (180/pi)*xGoal(1), 'r', 'Linewidth', 2.0);
% subplot(211), plot((1:N_tot)*dt, (180/pi)*testgoal1(1,:),'r', 'Linewidth', 2.0);
subplot(211), plot((1:N_tot)*dt, (180/pi)*xhat(2,1:N_tot), 'k', 'Linewidth', 2.0);
subplot(211), plot((1:N_tot)*dt, (180/pi)*xGoal(2), 'cy','Linewidth', 2.0);
% subplot(211), plot((1:N_tot)*dt, (180/pi)*testgoal2(1,:),'m', 'Linewidth', 2.0);
ylabel('Position (Degrees)')
legend('Joint 1 ', 'Joint 2');
title('DDP optimal weight lifting', 'FontSize',30)

subplot(212), plot((1:N_tot)*dt, uhat(1,1:N_tot), 'Linewidth', 2.0);
hold on;
subplot(212), plot((1:N_tot)*dt, uhat(2,:), 'c','Linewidth', 2.0);
ylabel('Control Input (Bar)');
xlabel('Time (s)');
legend('Input 1 ', 'Input 2');

%% PLOT of stiffness
figure(2)
subplot(211), plot((1:N_tot)*dt, Tnet(1,1:N_tot),  'Linewidth', 2.0);
hold on;
grid on;
subplot(211), plot((1:N_tot)*dt, Tnet(2,1:N_tot), 'r', 'Linewidth', 2.0);
ylabel('Net Torque (Degrees)')
legend('Joint 1 ', 'Joint 2');
title('Net Joint Torues: 0ptimal weight lifting', 'FontSize',30)

subplot(212), plot((1:N_tot)*dt, s(1,1:N_tot), 'Linewidth', 2.0);
hold on;
subplot(212), plot((1:N_tot)*dt, s(2,1:N_tot), 'r','Linewidth', 2.0);
ylabel('Stifness');
xlabel('Time (s)');
legend('Joint 1 ', 'Joint 2');


%%

% figure(5)
% plot((1:N_tot)*dt, L1k(1,:), 'Linewidth', 2.0);
% hold on;
% plot((1:N_tot)*dt, L2k(1,:),'r', 'Linewidth', 2.0);
% ylabel('Gain at X1');
% xlabel('Time (s)');
% title('Gain Matrix', 'FontSize',30)

% figure(6)
% subplot(211), plot((1:N_tot)*dt, 1e-5*x1test(5,:), 'Linewidth', 2.0);
% hold on;
% subplot(211), plot(, 1e-5*x1test(6,:), 'r', 'Linewidth', 2.0);
% ==== graphics ====

%function y = car_dynamics(x,u)
function y = pneumatics6st_dynamics(x,u)

%%
n_joint = 2; % number of joints in the manipulator;
% q = x(1:2,1);
% q_dot = x(3:4,1);
%state_deriv = zeros(8,1);
%Link 1 parameters
link1_lc = 125.4e-3;
link1_l = 351.1e-3;
m1 = 2.7;
link1_I = 0.02;
% link1_lc = 178e-3;
% link1_l = 307e-3;
% m1 = 2.578;
% link1_I = 0.0144;
% %Link 2 parameters
link2_lc = 178e-3;
link2_l = 307e-3;
m2 = 2.578;
link2_I = 0.0144;

%External load parametrs
mb = 0.5;

dt = 0.005;
%jointstate_deriv = zeros(6,1);
%joint_state = x;

% F = zeros(2,1);
% V = zeros(2,1);

%Pstate_deriv = zeros(4,1);
%%
theta1 = x(1,:,:);
theta2 = x(2,:,:);
% theta_dot = x(3,:);
Pdes1 = u(1,:,:);
Pdes2 = u(2,:,:); %bsxfun(@minus, 4e5, u(1,:,:));

%% Parameters for the muscles at Joint 1
p1 = -0.009338;   %(-0.01208, -0.006597)
p2 = 0.01444;
joint1_R = p1*x(1,1) + p2;
joint1_lo = 0.23;
joint1_alphaob = 20.0*pi/180;
joint1_alphaot = 20.0*pi/180;
joint1_k = 1.1;
joint1_ro = 0.012;
%joint1_R = 0.0095;
fv1 = 3.0;
wnb1 = omegacal(theta1,joint1_lo,joint1_alphaob,joint1_k,joint1_ro,joint1_R);

%% Parameters for the muscles at Joint 2
joint2_lo = 0.185;
joint2_alphaob = 23.0*pi/180;
joint2_alphaot = 23.0*pi/180;
joint2_k = 1.25;
joint2_ro = 0.0085;
joint2_R = 0.015;
fv2 = 0.25;
wnb2 = omegacal(theta2,joint2_lo,joint2_alphaob,joint2_k,joint2_ro,joint2_R);

wn = [wnb1 wnb2]';
%% Delta P Pressure Dynamics
%%%%%%% 2nd order  %%%%%%%%%%%%%%%
state_deriv(5,:) = x(7,:);
state_deriv(6,:) = x(8,:);
state_deriv(7,:) = (-wnb1.^2).*x(5,:,:) - 2*wnb1.*x(7,:,:) + (wnb1.^2).*Pdes1;
state_deriv(8,:) = (-wnb2.^2).*x(6,:,:) - 2*wnb2.*x(8,:,:) + (wnb2.^2).*Pdes2;

%% Force calculation
T1 = forcecal(x,joint1_lo,joint1_alphaob,joint1_k,joint1_ro,joint1_R,1,3);
T2 = forcecal(x,joint2_lo,joint2_alphaob,joint2_k,joint2_ro,joint2_R,2,4);
% T = [T1 T2]';

%% Mass Inertia Matrix 
m11_const = link1_I + m1*(link1_lc)^2 + link2_I + m2*(link1_l^2 + link2_lc^2) + mb*(link1_l^2 + link2_l^2);
m11_var = m2*2*link1_l*link2_lc.*cos(x(2,:)) + mb*2*link1_l*link2_l.*cos(x(2,:));
m11 = pp(m11_var,m11_const);

m12_const = link2_I + m2*link2_lc^2 + mb*link2_l^2;
m12_var = m2*link1_l*link2_lc.*cos(x(2,:)) + mb*link1_l*link2_l.*cos(x(2,:));
m12 = pp(m12_var,m12_const);
col = size(m12,2);
m22 = link2_I + m2*link2_lc^2 + mb*link2_l^2;
for k=1:col
    M(1:2,1:2,k) = [m11(1,k) m12(1,k);m12(1,k) m22];
end
% sm = size(M)
%M() = [m11 m12;m12 m22];
%% Coriolis Matrix
c1_const = -(m2*link2_lc + mb*link2_l)*link1_l;
c1_var1 = sin(x(2,:));
c1_var2 = 2*x(3,:).*x(4,:) + x(4,:).^2;
c1 = c1_const.*tt(c1_var1,c1_var2);

c2_const = (m2*link2_lc + mb*link2_l)*link1_l;
c2_var1 = sin(x(2,:));
c2_var2 = x(3,:).^2;
c2 = c2_const.*tt(c2_var1,c2_var2);
% C = [c1 c2]';
%% Gravity Matrix
g1 = (m1*link1_lc + m2*link1_l + mb*link1_l).*sin(x(1,:)) + (m2*link2_lc + mb*link2_l).*sin(x(1,:) + x(2,:));
g2 = (m2*link2_lc + mb*link2_l).*sin(x(1,:) + x(2,:));
sg = size(g1,2);
%% viscous friction matrix
tf1 = -fv1.*x(3,:);
tf2 = -fv2.*x(4,:);


for k=1:sg
    T(1:2,1,k) = [T1(1,k);T2(1,k)];
    G(1:2,1,k) = 9.8.*[g1(1,k);g2(1,k)];
    C(1:2,1,k) = [c1(1,k);c2(1,k)];
    Tf(1:2,1,k) = [tf1(1,k);tf2(1,k)];
end
%G = 9.8.*[g1 g2]';
% sg = size(G);
% st =size(T)
% sc =size(C)
%% Joint Dynamics
% Mat = [q1_dotdot, q2_dotdot]'
for k=1:col
  Mat(:,1,k)   = inv(M(:,:,k))*(T(:,:,k)+Tf(:,:,k) - C(:,:,k) - G(:,:,k));
end 
state_deriv(1,:) = x(3,:); %joint_state(2);
state_deriv(2,:) = x(4,:); %joint_state(2);
state_deriv(3,:) = Mat(1,:);
state_deriv(4,:) = Mat(2,:);
%((F_biceps -F_triceps ).*R  - fv.*theta_dot - (m*g*0.5*link_l).*sin(theta))/I;

y = x + dt.*state_deriv;
%y = state_deriv;
%% END

function c = pneumatic_cost(x, u, xGoal)
% cost function for car-parking problem
% sum of 3 terms:
% lu: quadratic cost on controls
% lf: final cost on distance from target parking configuration
% lx: small running cost on distance from origin to encourage tight turns
%x2dot = xGoal(9:10,1);
dt = 0.005;

xGoaltemp = xGoal;
xGoaltemp = xGoal;
%h = [dx1 dx1 dx1 dx1 dx2 dx2 dx2 dx2 dx1 dx1]';
% href = h(5:6,1); %Pxmat(h)
[Pref,Pref_dot] = Pxmat(xGoaltemp);
% xGoaltemp = xGoal + [h;0;0]
% Pref2 = Pxmat(xGoaltemp)
% xGoaltemp = xGoal - [h;0;0]
% Pref1 = Pxmat(xGoaltemp)
% Pref_Deriv(1) = (Pref2(1) - Pref1(1))/2*href(1,1)
% Pref_Deriv(2) = (Pref2(2) - Pref1(2))/2*href(2,1)
% Pxfun = @(xGoal) Pxmat(xGoal);
% Pref_Deriv = finite_difference(Pxfun,xGoal,dt)
% xGoal(5,1) = Pref(1,1);
% xGoal(6,1) = Pref(2,1);
% xGoal(7,1) = Pref_dot(1,1);
% xGoal(8,1) = Pref_dot(2,1);
goal(1:8,1) = xGoal(1:8,1); %[0.5;0;1e5;0];
final = isnan(u(1,:));
u(:,final)  = 0;

cu  = 1*1e-1*[0.001 .001];         % control cost coefficients

cf  = 8e-2*[1 1 0 0 0 0 0 0];    % final cost coefficients
cf2 = 1e-1;
%pf  = [.01 .01.01 0 1 0]';    % smoothness scales for final cost

cx  = 8e-2*[1 1 0 0 0 0 0 0];          % running cost coefficients
%px  = [.1 .1]';             % smoothness scales for running cost

% control cost
u;
lu    = cu*u.^2;
%l     = E.cx(1)*(x(1,:)-goal(1,1)).^2 +E.cx(2)*(x(2,:)-goal(2,1)).^2 +E.cx(3)*(x(3,:)-goal(3,1)).^2 +E.cx(4)*(x(4,:)-goal(4,1)).^2 +E.cx(5)*(x(5,:)-goal(5,1)).^2 +E.cx(6)*(x(6,:)-goal(6,1)).^2);

% final cost
if any(final)
   %llf      = cf*(x(:,final) - goal); %cf*sabs(x(:,final),pf);
   %llf      = cf* (bsxfun(@minus, x(:,final), goal)).^2;
   llf      = -cf2* x(4,final).^2;
   lf       = real(final);
   lf(final)= llf;
else
   lf    = 0;
end
lf;
%  running cost
for r=1:1:8
curcos(r,:) = cx(r).*(mm(x(r,:),goal(r,1))).^2;
end
curcos;
lx = sum(curcos);

% total const
c = lf;

function y = sabs(x,p)
% smooth absolute-value function (a.k.a pseudo-Huber)
y = pp( sqrt(pp(x.^2,p.^2)), -p);


function [f,c,fx,fu,fxx,fxu,fuu,cx,cu,cxx,cxu,cuu] = pneumatic_dyn_cst(x,u,full_DDP,xGoal)
% combine car dynamics and cost
% use helper function finite_difference() to compute derivatives
dt = 0.005;
if nargout == 2
    f = pneumatics6st_dynamics(x,u);
    c = pneumatic_cost(x,u,xGoal);
else
    % state and control indices
    ix = 1:8;
    iu = 9:10;
    
    % dynamics derivatives
    xu_dyn  = @(xu) pneumatics6st_dynamics(xu(ix,:),xu(iu,:));
    J       = finite_difference(xu_dyn, [x; u], dt);
    fx      = J(:,ix,:);
    fu      = J(:,iu,:);
    
    % cost first derivatives
    xu_cost = @(xu) pneumatic_cost(xu(ix,:),xu(iu,:),xGoal);
    J       = squeeze(finite_difference(xu_cost, [x; u], dt));
    cx      = J(ix,:);
    cu      = J(iu,:);
    
    % cost second derivatives
    xu_Jcst = @(xu) squeeze(finite_difference(xu_cost, xu, dt));
    JJ      = finite_difference(xu_Jcst, [x; u], dt);
    cxx     = JJ(ix,ix,:);
    cxu     = JJ(ix,iu,:);
    cuu     = JJ(iu,iu,:);
    
    % dynamics second derivatives
    if full_DDP
        xu_Jcst = @(xu) finite_difference(xu_dyn, xu, dt);
        JJ      = finite_difference(xu_Jcst, [x; u], dt);
        JJ      = reshape(JJ, [8 10 size(J)]);
        JJ      = 0.5*(JJ + permute(JJ,[1 3 2 4]));
        fxx     = JJ(:,ix,ix,:);
        fxu     = JJ(:,ix,iu,:);
        fuu     = JJ(:,iu,iu,:);    
    else
        [fxx,fxu,fuu] = deal([]);
    end
    
    [f,c] = deal([]);
end


function J = finite_difference(fun, x, h)
% simple finite-difference derivatives
% assumes the function fun() is vectorized

if nargin < 3
    h = 2^-17;
end

[n, K]  = size(x);
H       = [zeros(n,1) h*eye(n)];
H       = permute(H, [1 3 2]);
X       = pp(x, H);
X       = reshape(X, n, K*(n+1));
Y       = fun(X);
m       = numel(Y)/(K*(n+1));
Y       = reshape(Y, m, K, n+1);
J       = pp(Y(:,:,2:end), -Y(:,:,1)) / h;
J       = permute(J, [1 3 2]);


%% Sub function for system dynamic
function wnb = omegacal(theta,lo,alphaob,k,ro,R)
%theta = 0;
a_biceps = 3/(tan(alphaob))^2;
b_biceps = 1/(sin(alphaob))^2;
emax_biceps = (1/k)*(1 - sqrt(b_biceps/a_biceps));
alphaot = alphaob;
a_triceps = 3/(tan(alphaot))^2;
b_triceps = 1/(sin(alphaot))^2;
emax_triceps = (1/k)*(1 - sqrt(b_triceps/a_triceps));

lb = lo - R.*theta;
epsb = (1-(lb./lo));
lt = lo*(1-emax_triceps) + R.*theta;
epst = (1-(lt./lo));

%% Parameters of Joint
% m = 2.6;
% link_l = 0.32;
% g =9.81;
% I = m*(link_l^2)/3;
% fv = 0.25;
%% Volume calcuation
% biceps agonistic muscle
lb = lo - R.*theta;
csb2 = (cos(alphaob))^2;
epsb2 = (1-(lb./lo)).^2;
termb1 = (1 - csb2.*epsb2);
element1 = lb.*(pi*ro^2/((sin(alphaob))^2));
element2 = termb1.*1e6;
Vb = tt(element1,element2);
%Vb = 1630;
wnb = 2*pi*380.*(1./Vb);

% triceps antagonistic muscle 
lt = lo*(1-emax_triceps) + R.*theta; 
cst2 = (cos(alphaot))^2;
epst2 = (1-(lt./lo)).^2;
termt1 = (1 - cst2.*epst2);
elementt1 = lt.*(pi*ro^2/((sin(alphaot))^2));
elementt2 = termt1.*1e6;
Vt = tt(elementt1,elementt2);

%Vt = 1e6*(pi*lt*ro^2/((sin(alphaot))^2))*termt1
%Vt = 1630;
wnt = 2*pi*380*(1./Vt);

V = [Vb;Vt];

function Torqe_pneumatics = forcecal(x,lo,alphaob,k,ro,R,i,pmax)
theta = x(i,:,:);
a_biceps = 3/(tan(alphaob))^2;
b_biceps = 1/(sin(alphaob))^2;
emax_biceps = (1/k)*(1 - sqrt(b_biceps/a_biceps));
alphaot = alphaob;
a_triceps = 3/(tan(alphaot))^2;
b_triceps = 1/(sin(alphaot))^2;
emax_triceps = (1/k)*(1 - sqrt(b_triceps/a_triceps));

lb = lo - R.*theta;
epsb = (1-(lb./lo));
lt = lo*(1-emax_triceps) + R.*theta;
epst = (1-(lt./lo));


P1 = x(i+4,:,:);
P2 = bsxfun(@minus, pmax, x(i+4,:,:)); %x(4,:,:);
P = [P1;P2];
fbterm = 1e5*pi*ro^2*(a_biceps*(1-k.*epsb).^2 - b_biceps);
F_biceps =  P1.*fbterm;
ftterm = 1e5*pi*ro^2*(a_triceps*(1-k.*epst).^2 - b_triceps);
F_triceps = P2.*ftterm;
%F2max = 1*pi*ro^2*4*1e5*(a*(1-k*emax)^2 - b);
Fmat = [F_biceps; F_triceps];
Torqe_pneumatics = (F_biceps -F_triceps ).*R;

function [PxMat,PxMat_dot] = Pxmat(x)
accl1 = x(9,1);
accl2 = x(10,1);
jerk1 = x(11,1);
jerk2 = x(12,1);
%% Parameters
%Link 1 parameters
link1_lc = 125.4e-3;
link1_l = 351.1e-3;
m1 = 2.7;
link1_I = 0.02;
% link1_lc = 178e-3;
% link1_l = 307e-3;
% m1 = 2.578;
% link1_I = 0.0144;
% %Link 2 parameters
link2_lc = 178e-3;
link2_l = 307e-3;
m2 = 2.578;
link2_I = 0.0144;

%External load parametrs
mb = 0.5;

dt = 0.005;
%% Parameters for the muscles at Joint 1
p1 = -0.009338;   %(-0.01208, -0.006597)
p2 = 0.01444;
joint1_R = p1*x(1,1) + p2;
joint1_lo = 0.23;
joint1_alphaob = 20.0*pi/180;
joint1_alphaot = 20.0*pi/180;
joint1_k = 1.1;
joint1_ro = 0.012;
% joint1_R = 0.0095;
fv1 = 3.0;
%wnb1 = omegacal(theta1,joint1_lo,joint1_alphaob,joint1_k,joint1_ro,joint1_R);

%% Parameters for the muscles at Joint 2
joint2_lo = 0.185;
joint2_alphaob = 23.0*pi/180;
joint2_alphaot = 23.0*pi/180;
joint2_k = 1.25;
joint2_ro = 0.0085;
joint2_R = 0.015;
fv2 = 0.25;
%wnb2 = omegacal(theta2,joint2_lo,joint2_alphaob,joint2_k,joint2_ro,joint2_R);

%wn = [wnb1 wnb2]';

%% Muslces force parameter calculation
[fx1term,Pst1,fx1term_dot,Pst1_dot] = fxparcal(x,joint1_lo,joint1_alphaob,joint1_k,joint1_ro,joint1_R,1,3);
[fx2term,Pst2,fx2term_dot,Pst2_dot] = fxparcal(x,joint2_lo,joint2_alphaob,joint2_k,joint2_ro,joint2_R,2,4);
% T = [T1 T2]';

%% Mass Inertia Matrix 
m11_const = link1_I + m1*(link1_lc)^2 + link2_I + m2*(link1_l^2 + link2_lc^2) + mb*(link1_l^2 + link2_l^2);
m11_var = m2*2*link1_l*link2_lc.*cos(x(2,:)) + mb*2*link1_l*link2_l.*cos(x(2,:));
m11 = pp(m11_var,m11_const);

m11_dot = m2*2*link1_l*link2_lc.*(-sin(x(2,:))).*x(4,:) + mb*2*link1_l*link2_l.*(-sin(x(2,:))).*x(4,:);

m12_const = link2_I + m2*link2_lc^2 + mb*link2_l^2;
m12_var = m2*link1_l*link2_lc.*cos(x(2,:)) + mb*link1_l*link2_l.*cos(x(2,:));
m12 = pp(m12_var,m12_const);

m12_dot = m2*link1_l*link2_lc.*(-sin(x(2,:))).*x(4,:) + mb*link1_l*link2_l.*(-sin(x(2,:))).*x(4,:);

m22_dot =0;
col = size(m12,2);
m22 = link2_I + m2*link2_lc^2 + mb*link2_l^2;
for k=1:col
    M(1:2,1:2,k) = [m11(1,k) m12(1,k);m12(1,k) m22];
    Fxterm(1:2,1:2,k) = [fx1term(1,k) 0;0 fx2term(1,k)];
    M_dot(1:2,1:2,k) = [m11_dot(1,k) m12_dot(1,k);m12_dot(1,k) m22_dot];
    Fxterm_dot(1:2,1:2,k) = [fx1term_dot(1,k) 0;0 fx2term_dot(1,k)];
end
% sm = size(M)
%M() = [m11 m12;m12 m22];
%% Coriolis Matrix
c1_const = -(m2*link2_lc + mb*link2_l)*link1_l;
c1_var1 = sin(x(2,:));
c1_var2 = 2*x(3,:).*x(4,:) + x(4,:).^2;
c1 = c1_const.*tt(c1_var1,c1_var2);

c2_const = (m2*link2_lc + mb*link2_l)*link1_l;
c2_var1 = sin(x(2,:));
c2_var2 = x(3,:).^2;
c2 = c2_const.*tt(c2_var1,c2_var2);

% C1_dot
c1dot_var1_1 = cos(x(2,:)).*x(4,:);
c1dot_var1_2 = 2*x(3,:).*x(4,:) + x(4,:).^2;
c1dot_1 = c1_const.*tt(c1dot_var1_1,c1dot_var1_2);

c1dot_var2 = 2.*accl1.*x(4,:) + 2*x(3,:).*accl2 + 2.*x(4,:).*accl2;
c1dot_2 = c1_const.*tt(c1_var1,c1dot_var2);
c1_dot = pp(c1dot_1,c1dot_2);

%C2_dot
c2dot_var1_1 = cos(x(2,:));
c2dot_var1_2 =  x(4,:).*(x(3,:).^2);
c2dot_1 = c2_const.*tt(c2dot_var1_1,c2dot_var1_2);

c2dot_var2 = 2*x(3,:).*accl1;
c2dot_2 = c2_const.*tt(c2_var1,c2dot_var2);
c2_dot = pp(c2dot_1,c2dot_2);

% C = [c1 c2]';
%% Gravity Matrix
g1 = (m1*link1_lc + m2*link1_l + mb*link1_l).*sin(x(1,:)) + (m2*link2_lc + mb*link2_l).*sin(x(1,:) + x(2,:));
g2 = (m2*link2_lc + mb*link2_l).*sin(x(1,:) + x(2,:));

g1_dot = (m1*link1_lc + m2*link1_l + mb*link1_l).*cos(x(1,:)).*x(3,:) + (m2*link2_lc + mb*link2_l).*cos(x(1,:) + x(2,:)).*(x(3,:) + x(4,:));
g2_dot = (m2*link2_lc + mb*link2_l).*cos(x(1,:) + x(2,:)).*(x(3,:) + x(4,:));

sg = size(g1,2);
%% viscous friction matrix
tf1 = fv1.*x(3,:);
tf2 = fv2.*x(4,:);
tf1_dot = fv1.*accl1;
tf2_dot = fv2.*accl2;

for k=1:sg
    %T(1:2,1,k) = [T1(1,k);T2(1,k)];
    G(1:2,1,k) = 9.8.*[g1(1,k);g2(1,k)];
    C(1:2,1,k) = [c1(1,k);c2(1,k)];
    Tf(1:2,1,k) = [tf1(1,k);tf2(1,k)];
    Pst(1:2,1,k) = [Pst1(1,k);Pst2(1,k)];
    
    G_dot(1:2,1,k) = 9.8.*[g1_dot(1,k);g2_dot(1,k)];
    C_dot(1:2,1,k) = [c1_dot(1,k);c2_dot(1,k)];
    Tf_dot(1:2,1,k) = [tf1_dot(1,k);tf2_dot(1,k)];
    Pst_dot(1:2,1,k) = [Pst1_dot(1,k);Pst2_dot(1,k)];
end

for k=1:col
  %PxMat(:,1,k)   = inv(Fxterm(:,:,k))*(M(:,:,k)*x(9:10,1)+Tf(:,:,k) + C(:,:,k) + G(:,:,k) + Pst(:,:,k));
 
  
  PxMat(1,1,k) = (1/fx1term(1,k))*(m11*x(9,1) + m12*x(10,1) + Tf(1,1,k) +C(1,1,k) + G(1,1,k) + Pst(1,1,k));
  PxMat(2,1,k) = (1/fx2term(1,k))*(m12*x(9,1) + m22*x(10,1) + Tf(2,1,k) +C(2,1,k) + G(2,1,k) + Pst(2,1,k));
  
  %PxMat_dot(:,1,k)   = inv(Fxterm(:,:,k))*(M(:,:,k)*x(11:12,1) + M_dot(:,:,k)*x(9:10,1) + Tf_dot(:,:,k) + C_dot(:,:,k) + G_dot(:,:,k) + Pst_dot(:,:,k) - Fxterm_dot(:,:,k)*PxMat(:,1,k));  
  
  PxMat_dot(1,1,k) = (1/fx1term(1,k))*(m11*x(11,1) + m12*x(12,1) + m11_dot*x(9,1) + m12_dot*x(10,1) + Tf_dot(1,1,k) + C_dot(1,1,k) + G_dot(1,1,k) + Pst_dot(1,1,k) - Fxterm_dot(1,1,k)*PxMat(1,1,k));
  PxMat_dot(2,1,k) = (1/fx1term(1,k))*(m12*x(11,1) + m22*x(12,1) + m12_dot*x(9,1) + m22_dot*x(10,1) + Tf_dot(2,1,k) + C_dot(2,1,k) + G_dot(2,1,k) + Pst_dot(2,1,k) - Fxterm_dot(2,2,k)*PxMat(2,1,k));

end


function [fxterm, Pst, fxterm_dot, Pst_dot] = fxparcal(x,lo,alphaob,k,ro,R,i,pmax)
theta = x(i,:,:);
% if (i==1)
%     p1 = -0.009338;   %(-0.01208, -0.006597)
%     p2 = 0.01444;
%     R = p1*x(1,1) + p2;
% end
a_biceps = 3/(tan(alphaob))^2;
b_biceps = 1/(sin(alphaob))^2;
emax_biceps = (1/k)*(1 - sqrt(b_biceps/a_biceps));
alphaot = alphaob;
a_triceps = 3/(tan(alphaot))^2;
b_triceps = 1/(sin(alphaot))^2;
emax_triceps = (1/k)*(1 - sqrt(b_triceps/a_triceps));

lb = lo - R.*theta;
epsb = (1-(lb./lo));
lt = lo*(1-emax_triceps) + R.*theta;
epst = (1-(lt./lo));

P1 = x(i+4,:,:);
P2 = bsxfun(@minus, pmax, x(i+4,:,:)); %x(4,:,:);
P = [P1;P2];
fbterm = 1e5*R*pi*ro^2*(a_biceps*(1-k.*epsb).^2 - b_biceps);
fbterm_dot = 1e5*R*pi*ro^2*a_biceps*2*(-k)*(R/lo).*(1-k.*epsb).*x(i+2,:,:);
%F_biceps =  P1.*fbterm;
ftterm = 1e5*R*pi*ro^2*(a_triceps*(1-k.*epst).^2 - b_triceps);
ftterm_dot = 1e5*R*pi*ro^2*a_triceps*2*(k)*(R/lo).*(1-k.*epst).*x(i+2,:,:);
fxterm = fbterm + ftterm;
fxterm_dot = fbterm_dot + ftterm_dot;
Pst = pmax.*ftterm;
Pst_dot = pmax.*ftterm_dot;
%F_triceps = P2.*ftterm;
%F2max = 1*pi*ro^2*4*1e5*(a*(1-k*emax)^2 - b);
%Fmat = [F_biceps; F_triceps];
% Torqe_pneumatics = (F_biceps -F_triceps ).*R;
% utility functions, singleton-expanded addition and multiplication
%% Net Torque at Joint and stiffness calculation
function Net_T = Net_Torue(x,u)

%%
n_joint = 2; % number of joints in the manipulator;
% q = x(1:2,1);
% q_dot = x(3:4,1);
%state_deriv = zeros(8,1);
%Link 1 parameters
link1_lc = 125.4e-3;
link1_l = 351.1e-3;
m1 = 2.7;
link1_I = 0.02;
% link1_lc = 178e-3;
% link1_l = 307e-3;
% m1 = 2.578;
% link1_I = 0.0144;
% %Link 2 parameters
link2_lc = 178e-3;
link2_l = 307e-3;
m2 = 2.578;
link2_I = 0.0144;

%External load parametrs
mb = 0.5;

dt = 0.005;
%jointstate_deriv = zeros(6,1);
%joint_state = x;

% F = zeros(2,1);
% V = zeros(2,1);

%Pstate_deriv = zeros(4,1);
%%
theta1 = x(1,:,:);
theta2 = x(2,:,:);
% theta_dot = x(3,:);
Pdes1 = u(1,:,:);
Pdes2 = u(2,:,:); %bsxfun(@minus, 4e5, u(1,:,:));

%% Parameters for the muscles at Joint 1
p1 = -0.009338;   %(-0.01208, -0.006597)
p2 = 0.01444;
joint1_R = p1*x(1,1) + p2;
joint1_lo = 0.23;
joint1_alphaob = 20.0*pi/180;
joint1_alphaot = 20.0*pi/180;
joint1_k = 1.1;
joint1_ro = 0.012;
%joint1_R = 0.0095;
fv1 = 3.0;
wnb1 = omegacal(theta1,joint1_lo,joint1_alphaob,joint1_k,joint1_ro,joint1_R);

%% Parameters for the muscles at Joint 2
joint2_lo = 0.185;
joint2_alphaob = 23.0*pi/180;
joint2_alphaot = 23.0*pi/180;
joint2_k = 1.25;
joint2_ro = 0.0085;
joint2_R = 0.015;
fv2 = 0.25;
wnb2 = omegacal(theta2,joint2_lo,joint2_alphaob,joint2_k,joint2_ro,joint2_R);

wn = [wnb1 wnb2]';
%% Delta P Pressure Dynamics
%%%%%%% 2nd order  %%%%%%%%%%%%%%%
state_deriv(5,:) = x(7,:);
state_deriv(6,:) = x(8,:);
state_deriv(7,:) = (-wnb1.^2).*x(5,:,:) - 2*wnb1.*x(7,:,:) + (wnb1.^2).*Pdes1;
state_deriv(8,:) = (-wnb2.^2).*x(6,:,:) - 2*wnb2.*x(8,:,:) + (wnb2.^2).*Pdes2;

%% Force calculation
T1 = forcecal(x,joint1_lo,joint1_alphaob,joint1_k,joint1_ro,joint1_R,1,3);
T2 = forcecal(x,joint2_lo,joint2_alphaob,joint2_k,joint2_ro,joint2_R,2,4);
% T = [T1 T2]';

%% Mass Inertia Matrix 
m11_const = link1_I + m1*(link1_lc)^2 + link2_I + m2*(link1_l^2 + link2_lc^2) + mb*(link1_l^2 + link2_l^2);
m11_var = m2*2*link1_l*link2_lc.*cos(x(2,:)) + mb*2*link1_l*link2_l.*cos(x(2,:));
m11 = pp(m11_var,m11_const);

m12_const = link2_I + m2*link2_lc^2 + mb*link2_l^2;
m12_var = m2*link1_l*link2_lc.*cos(x(2,:)) + mb*link1_l*link2_l.*cos(x(2,:));
m12 = pp(m12_var,m12_const);
col = size(m12,2);
m22 = link2_I + m2*link2_lc^2 + mb*link2_l^2;
for k=1:col
    M(1:2,1:2,k) = [m11(1,k) m12(1,k);m12(1,k) m22];
end
% sm = size(M)
%M() = [m11 m12;m12 m22];
%% Coriolis Matrix
c1_const = -(m2*link2_lc + mb*link2_l)*link1_l;
c1_var1 = sin(x(2,:));
c1_var2 = 2*x(3,:).*x(4,:) + x(4,:).^2;
c1 = c1_const.*tt(c1_var1,c1_var2);

c2_const = (m2*link2_lc + mb*link2_l)*link1_l;
c2_var1 = sin(x(2,:));
c2_var2 = x(3,:).^2;
c2 = c2_const.*tt(c2_var1,c2_var2);
% C = [c1 c2]';
%% Gravity Matrix
g1 = (m1*link1_lc + m2*link1_l + mb*link1_l).*sin(x(1,:)) + (m2*link2_lc + mb*link2_l).*sin(x(1,:) + x(2,:));
g2 = (m2*link2_lc + mb*link2_l).*sin(x(1,:) + x(2,:));
sg = size(g1,2);
%% viscous friction matrix
tf1 = -fv1.*x(3,:);
tf2 = -fv2.*x(4,:);


for k=1:sg
    T(1:2,1,k) = [T1(1,k);T2(1,k)];
    G(1:2,1,k) = 9.8.*[g1(1,k);g2(1,k)];
    C(1:2,1,k) = [c1(1,k);c2(1,k)];
    Tf(1:2,1,k) = [tf1(1,k);tf2(1,k)];
end
%G = 9.8.*[g1 g2]';
% sg = size(G);
% st =size(T)
% sc =size(C)
%% Joint Dynamics
% Mat = [q1_dotdot, q2_dotdot]'
Net_T = (T(:,:,k)+Tf(:,:,k) - C(:,:,k) - G(:,:,k));



function c=pp(a,b)
c = bsxfun(@plus,a,b);

function c=tt(a,b)
c = bsxfun(@times,a,b);

function c=mm(a,b)
c = bsxfun(@minus,a,b);