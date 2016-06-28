function demo_linear
% A demo of iLQG/DDP with a control-limited LTI system.

fprintf(['A demonstration of the iLQG/DDP algorithm\n'...
'with a random control-limited time-invariant linear system.\n'...
'for details see\nTassa, Mansard & Todorov, ICRA 2014\n'...
'\"Control-Limited Differential Dynamic Programming\"\n'])

% make stable linear dynamics
h = .0001;        % time step
n = 4;         % state dimension
m = 1;          % control dimension
A =         [1   0.0001        0        0;
            -1.18116 0.927536        0 -72.4638;
             0        0        1   0.0001;
            0.001        0        0        1];


B =     [ 0;
            36.2319;
            0;
            0];


% quadratic costs
Q = [100,0,0,0;
    0,0,0,0;
    0,0,0,0;
    0,0,0,0;];
R = .1*eye(m);

% control limits
%Op.lims = ones(m,1)*[-1 1]*.6;

% optimization problem
DYNCST  = @(x,u,i) lin_dyn_cst(x,u,A,B,Q,R);
T       = 150;              % horizon
x0      = [-3.0;0.0;0.0;0.0];       % initial state
u0      = .1*randn(m,T);    % initial controls

% run the optimization
[x,u] = iLQG(DYNCST, x0, u0);

figure
subplot(221)
plot(x(1,:,:))
grid on
subplot(222)
plot(x(2,:,:))
grid on
subplot(223)
plot(x(3,:,:))
grid on
subplot(224)
plot(x(4,:,:))
grid on

figure
plot(u)
grid on





function [f,c,fx,fu,fxx,fxu,fuu,cx,cu,cxx,cxu,cuu] = lin_dyn_cst(x,u,A,B,Q,R)

% for a PD quadratic u-cost  
% no cost (nans) is equivalent to u=0
u(isnan(u)) = 0;

if nargout == 2
    f = A*x + B*u;
    c = 0.5*sum(x.*(Q*x),1) + 0.5*sum(u.*(R*u),1);
else 
    N   = size(x,2);
    fx  = repmat(A, [1 1 N]);
    fu  = repmat(B, [1 1 N]);
    cx  = Q*x;
    cu  = R*u;
    cxx = repmat(Q, [1 1 N]);
    cxu = repmat(zeros(size(B)), [1 1 N]);
    cuu = repmat(R, [1 1 N]);
    [f,c,fxx,fxu,fuu] = deal([]);
end