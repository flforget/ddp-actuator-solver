function demo_pneumatic
% A demo of iLQG/DDP with car-parking dynamics

fprintf(['\nA demonstration of the iLQG algorithm '...
'with car parking dynamics.\n'...
'for details see\nTassa, Mansard & Todorov, ICRA 2014\n'...
'\"Control-Limited Differential Dynamic Programming\"\n'])

% Set full_DDP=true to compute 2nd order derivatives of the 
% dynamics. This will make iterations more expensive, but 
% final convergence will be much faster (quadratic)
full_DDP = false;

N=1;
dt = 1; % 0.005;
sub_dt = 0.005;
T       = 200; %dt/sub_dt;              % horizon
x0      = [0;0;0e5;4e5];   % initial state
u0      = 10.1*randn(2,T);    % initial controls
% u0(1,1:T) = 0e5*ones(1,1:T);
% u0(2,1:T) = 4e5*ones(1,1:T);
Op.lims  = [0 4.5e5;         % muscle 1 pressure limits (pascal)
            0  4.5e5];       % muscle 2 pressure limits (pascal)
%Op.maxIter = 2;
% run the optimization
Op.plot = -1;
dt =0.005;


    % optimization problem
%     xgoal(1,1) = i*(45/N)*(pi/180);
%     xgoal(2,1) = (45/(N*dt))*(pi/180);
%     xgoal(3,1) = i*(2/N)*1e5;
%     xgoal(4,1) = 4e5 - i*(2/N)*1e5;
%     xgoal
    DYNCST  = @(x,u,i) pneumatic_dyn_cst(x,u,full_DDP);
    [x,u]= iLQG(DYNCST, x0, u0, Op);
%     x0 = x(:,end);
%     u0 = u(:,1);
%     xplot(:,i) = x0;
%     uplot(:,i) = u0;
     

figure(4)
subplot(221), plot(xplot(1,:));
subplot(222), plot(uplot(1,:));
hold on;
subplot(222), plot(uplot(2,:),'g');

subplot(223), plot(xplot(3,:));
subplot(224), plot(xplot(4,:));
% ==== graphics ====

%function y = car_dynamics(x,u)
function y = pneumatics6st_dynamics(x,u)

dt = 0.005;
%jointstate_deriv = zeros(6,1);
%joint_state = x;

F = zeros(2,1);
V = zeros(2,1);

%Pstate_deriv = zeros(4,1);
theta = x(1,:,:);
theta_dot = x(2,:,:);
Pdes1 = u(1,:,:);
Pdes2 = u(2,:,:);
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

lb = lo - R.*theta;
epsb = (1-(lb./lo));
lt = lo*(1-emax_triceps) + R.*theta;
epst = (1-(lt./lo));

%% Parameters of Joint
m = 2.6;
link_l = 0.32;
g =9.81;
I = m*(link_l^2)/3;
fv = 0.25;
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

%% Pressure Dynamics
%%%%%%% 2nd order  %%%%%%%%%%%%%%%
% jointstate_deriv(3,:) = x(4,:);
% %Pstate_deriv(2,:,:) = (-wnb.^2).*Pb_state(1,:,:) - 2*wnb.*Pb_state(2,:,:) + (wnb.^2).*Pdes1;
% jointstate_deriv(4,:) = (-wnb.^2).*x(3,:,:) - 2*wnb.*x(4,:,:) + (wnb.^2).*Pdes1;
% jointstate_deriv(5,:) = x(6,:,:);
% %Pstate_deriv(4,:,:) = -wnt.^2*Pt_state(1,:,:) - 2*wnt.*Pt_state(2,:,:) + (wnt.^2).*Pdes2;
% jointstate_deriv(6,:) = (-wnt.^2).*x(5,:,:) - 2*wnt.*x(5,:,:) + (wnt.^2).*Pdes2;
%%%%%%% 1st order %%%%%%%%%%%%%%%%
time_constant1 = 0.18;
time_constant2 = 0.13;
jointstate_deriv(3,:) = (-x(3,:)./time_constant1) + (Pdes1./time_constant1);

jointstate_deriv(4,:) = (-x(4,:)./time_constant2) + (Pdes2./time_constant2);



%% Force calculation
P1 = x(3,:,:);
P2 = x(4,:,:);
fbterm = pi*ro^2*(a_biceps*(1-k.*epsb).^2 - b_biceps);
F_biceps =  P1.*fbterm;
ftterm = pi*ro^2*(a_triceps*(1-k.*epst).^2 - b_triceps);
F_triceps = P2.*ftterm;
%F2max = 1*pi*ro^2*4*1e5*(a*(1-k*emax)^2 - b);
F = [F_biceps; F_triceps];

%% Joint Dynamics
jointstate_deriv(1,:) = theta_dot(1,:); %joint_state(2);
jointstate_deriv(2,:) = ((F_biceps -F_triceps ).*R  - fv.*theta_dot - (m*g*0.5*link_l).*sin(theta))./I;
%jointstate_deriv(3:6,:,:) = Pstate_deriv(1:4,:,:);
%dy = jointstate_deriv(:,1);
y = x + dt.*jointstate_deriv;

%% END



function c = pneumatic_cost(x, u)
% cost function for car-parking problem
% sum of 3 terms:
% lu: quadratic cost on controls
% lf: final cost on distance from target parking configuration
% lx: small running cost on distance from origin to encourage tight turns
goal = [1,0,2e5,2e5]';
final = isnan(u(1,:));
u(:,final)  = 0;

cu  = 1e-1*[0.01 .01];         % control cost coefficients

cf  = 1e-3*[ 1  0  1  1];    % final cost coefficients
%pf  = [.01 .01 .01 0 1 0]';    % smoothness scales for final cost

cx  = 1e-2*[1 0 1 1 ];          % running cost coefficients
%px  = [.1 .1]';             % smoothness scales for running cost

% control cost
lu    = cu*u.^2;
%l     = E.cx(1)*(x(1,:)-goal(1,1)).^2 +E.cx(2)*(x(2,:)-goal(2,1)).^2 +E.cx(3)*(x(3,:)-goal(3,1)).^2 +E.cx(4)*(x(4,:)-goal(4,1)).^2 +E.cx(5)*(x(5,:)-goal(5,1)).^2 +E.cx(6)*(x(6,:)-goal(6,1)).^2);

% final cost
if any(final)
   llf      = cf*(x(:,end) - goal); %cf*sabs(x(:,final),pf);
   lf       = real(final);
   lf(final)= llf;
else
   lf    = 0;
end

% running cost
% for r=1:1:6
% lx1(r) = cx(r)*(x(r,:) - goal(r,1)).^2;
% end

lx1 = cx(1)*(x(1,:) - goal(1,1)).^2;
lx2 = cx(2)*(x(2,:) - goal(2,1)).^2;
lx3 = cx(3)*(x(3,:) - goal(3,1)).^2;
lx4 = cx(4)*(x(4,:) - goal(4,1)).^2;
% lx5 = cx(5)*(x(5,:) - goal(5,1)).^2;
% lx6 = cx(6)*(x(6,:) - goal(6,1)).^2;
%lx = cx*(x-goal);%cx*sabs(x(1:2,:),px);
lx = lx1 + lx2 + lx3 + lx4; % +lx5 + lx6;
% total const
c     = lu + lx + lf;

function y = sabs(x,p)
% smooth absolute-value function (a.k.a pseudo-Huber)
y = pp( sqrt(pp(x.^2,p.^2)), -p);


function [f,c,fx,fu,fxx,fxu,fuu,cx,cu,cxx,cxu,cuu] = pneumatic_dyn_cst(x,xgoal,u,full_DDP)
% combine car dynamics and cost
% use helper function finite_difference() to compute derivatives
dt = 0.005;
if nargout == 2
    f = pneumatics6st_dynamics(x,u);
    c = pneumatic_cost(x,xgoal,u);
else
    % state and control indices
    ix = 1:4;
    iu = 5:6;
    
    % dynamics derivatives
    xu_dyn  = @(xu) pneumatics6st_dynamics(xu(ix,:),xu(iu,:));
    J       = finite_difference(xu_dyn, [x; u], dt);
    fx      = J(:,ix,:);
    fu      = J(:,iu,:);
    
    % cost first derivatives
    xu_cost = @(xu) pneumatic_cost(xu(ix,:),xgoal,xu(iu,:));
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
        JJ      = reshape(JJ, [4 6 size(J)]);
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



% utility functions, singleton-expanded addition and multiplication
function c=pp(a,b)
c = bsxfun(@plus,a,b);

function c=tt(a,b)
c = bsxfun(@times,a,b);