
% x = [q1 q2 q1_dot q2_dot delP1 delP2 delP1_dot delP2_dot]';
% u = [u1 u2]';
%
%
%
%

function y = pneumatics2link_dynamics(x,u)
n_joint = 2; % number of joints in the manipulator;
q = x(1:2,1);
q_dot = x(3:4,1);

%Link 1 parameters
link1_lc = 125.4e-3;
link1_l = 351.1e-3;
m1 = 2.7;
link1_I = 0.02;

%Link 2 parameters
link2_lc = 178e-3;
link2_l = 317e-3;
m2 = 2.578;
link2_I = 0.0144;

%External load parametrs
mb = 0.01;

dt = 0.005;
%jointstate_deriv = zeros(6,1);
%joint_state = x;

% F = zeros(2,1);
% V = zeros(2,1);

%Pstate_deriv = zeros(4,1);
theta = x(1,:,:);
theta_dot = x(3,:,:);
Pdes1 = u(1,:,:);
Pdes2 = u(2,:,:); %bsxfun(@minus, 4e5, u(1,:,:));

%% Parameters for the muscles at Joint 1
joint1_lo = 0.23;
joint1_alphaob = 23.0*pi/180;
joint1_alphaot = 23.0*pi/180;
joint1_k = 1.1;
joint1_ro = 0.009;
joint1_R = 0.009;
fv1 = 0.25;
wnb1 = omegacal(theta,joint1_lo,joint1_alphaob,joint1_k,joint1_ro,joint1_R);

%% Parameters for the muscles at Joint 2
joint2_lo = 0.185;
joint2_alphaob = 23.0*pi/180;
joint2_alphaot = 23.0*pi/180;
joint2_k = 1.25;
joint2_ro = 0.009;
joint2_R = 0.015;
fv2 = 0.25;
wnb2 = omegacal(theta,joint2_lo,joint2_alphaob,joint2_k,joint2_ro,joint2_R);


%% Delta P Pressure Dynamics
%%%%%%% 2nd order  %%%%%%%%%%%%%%%
state_deriv(5,:) = x(7,:);
state_deriv(6,:) = x(8,:);
state_deriv(7,:) = (-wnb1.^2).*x(5,:,:) - 2*wnb1.*x(7,:,:) + (wnb1.^2).*Pdes1;
state_deriv(8,:) = (-wnb2.^2).*x(6,:,:) - 2*wnb2.*x(8,:,:) + (wnb2.^2).*Pdes2;

%% Force calculation
T1 = forcecal(theta,joint1_lo,joint1_alphaob,joint1_k,joint1_ro,joint1_R);
T2 = forcecal(theta,joint2_lo,joint2_alphaob,joint2_k,joint2_ro,joint2_R);
T = [T1 T2]';
%% Mass Inertia Matrix 
m11_const = link1_I + m1*(link1_lc)^2 + link2_I + m2*(link1_l^2 + link2_lc^2) + mb*(link1_l^2 + link2_l^2);
m11_var = m2*2*link1_l*link2_lc.*cos(x(2,:)) + mb*2*link1_l*link2_l.*cos(x(2,:));
m11 = pp(m11_var,m11_const);

m12_const = link2_I + m2*link2_lc^2 + mb*link2_l^2;
m12_var = m2*link1_l*link2_lc.*cos(x(2,:)) + mb*link1_l*link2_l.*cos(x(2,:));
m12 = pp(m12_var,m12_const);

m22 = link2_I + m2*link2_lc^2 + mb*link2_l^2;
M = [m11 m12;m12 m22];
%% Coriolis Matrix
c1_const = -(m2*link2_lc + mb*link2_l)*link1_l;
c1_var1 = sin(x(2,:));
c1_var2 = 2*x(3,:).*x(4,:) + x(4,:).^2;
c1 = c1_const.*c1_var1.*c1_var2;

c2_const = -(m2*link2_lc + mb*link2_l)*link1_l;
c2_var1 = sin(x(2,:));
c2_var2 = x(3,:).^2;
c2 = c2_const.*c2_var1.*c2_var2;
C = [c1 c2]';
%% Gravity Matrix
g1 = (m1*link1_lc + m2*link1_l + mb*link1_l).*cos(x(1,:)) + (m2*link2_lc + mb*link2_l).*cos(x(1,:) + x(2,:));
g2 = (m2*link2_lc + mb*link2_l).*cos(x(1,:) + x(2,:));
G = [g1 g2]';
%% Joint Dynamics
tf1 = - fv1.*x(3,:);
tf2 = - fv2.*x(4,:);
Tf = [tf1 tf2]';

  Mat   = M'*(T+Tf - C-G);
  
state_deriv(1,:) = x(3,:); %joint_state(2);
state_deriv(2,:) = x(4,:); %joint_state(2);
state_deriv(3,:) = Mat(1,:);
state_deriv(4,:) = Mat(2,:);
%((F_biceps -F_triceps ).*R  - fv.*theta_dot - (m*g*0.5*link_l).*sin(theta))/I;

y = x + dt.*state_deriv;
end
%% END

function wnb = omegacal(theta,lo,alphaob,k,ro,R)

a_biceps = 3/(tan(alphaob))^2;
b_biceps = 1/(sin(alphaob))^2;
emax_biceps = (1/k)*(1 - sqrt(b_biceps/a_biceps));

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
wnb = 2*pi*380*(1./Vb);

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
end
function Torqe_pneumatics = forcecal(theta,lo,alphaob,k,ro,R)

a_biceps = 3/(tan(alphaob))^2;
b_biceps = 1/(sin(alphaob))^2;
emax_biceps = (1/k)*(1 - sqrt(b_biceps/a_biceps));

a_triceps = 3/(tan(alphaot))^2;
b_triceps = 1/(sin(alphaot))^2;
emax_triceps = (1/k)*(1 - sqrt(b_triceps/a_triceps));

lb = lo - R.*theta;
epsb = (1-(lb./lo));
lt = lo*(1-emax_triceps) + R.*theta;
epst = (1-(lt./lo));


P1 = x(3,:,:);
P2 = bsxfun(@minus, 4e5, x(3,:,:)); %x(4,:,:);
P = [P1;P2];
fbterm = pi*ro^2*(a_biceps*(1-k.*epsb).^2 - b_biceps);
F_biceps =  P1.*fbterm;
ftterm = pi*ro^2*(a_triceps*(1-k.*epst).^2 - b_triceps);
F_triceps = P2.*ftterm;
%F2max = 1*pi*ro^2*4*1e5*(a*(1-k*emax)^2 - b);
Fmat = [F_biceps; F_triceps];
Torqe_pneumatics = (F_biceps -F_triceps ).*R;
end

function c=pp(a,b)
c = bsxfun(@plus,a,b);

function c=tt(a,b)
c = bsxfun(@times,a,b);