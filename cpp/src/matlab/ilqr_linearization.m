function [A, B] = ilqr_linearization(x, u, dt)

% [A, B] = ilqr_linearization(x, u, dt)
%
% linearizes the system in preparation to apply the ilqr techniques.
%
% given	-> x the state of the system at some time
%	-> u the command into the system at some time
%	-> dt the current time step used
%
% returns -> A the current jacobian Dxf(x,u) as defined by todorov
%	  -> B the current jacobian Duf(x,u) as defined by todorov
%
% created by timothy lillicrap (tim@biomed.queensu.ca); december 1, 2005
% last edited by timothy lillicrap (tim@biomed.queensu.ca); december 1, 2005

A = zeros(length(x), length(x));
B = zeros(length(x), length(u));

% g = 9.8;
% m = 1.0;
% l = 1.0;
% mu = 0.01;

% A = diag([-10.73*dt 0 0 ]);
% A(1,2) = -84.23*dt;
% A(1,3) = -366*dt;
% A(2,1) = 1*dt; 
% A(2,3) = 0;
% A(3,1) = 0;
% A(3,2) = 1*dt;
% 
% B(1,1) = 1*dt;
% B(2,1) = 0;
% B(3,1) = 0;

%% Continuous Time model

% A = diag([(1-10.73*dt) 1 1]);
% A(1,2) = -10.53*dt;
% A(1,3) = -5.719*dt;
% A(2,1) = 8*dt; 
% A(2,3) = 0;
% A(3,1) = 0;
% A(3,2) = 8*dt;
% 
% B(1,1) = 2*dt;
% B(2,1) = 0;
% B(3,1) = 0;


%% Continuous Time modelfrm Transfer function

% A = diag([ 1 1 (1-10.73*dt)]);
% A(1,2) = dt;
% A(1,3) = 0;
% A(2,1) = 0; 
% A(2,3) = dt;
% A(3,1) = -366*dt;
% A(3,2) = -84.23*dt;
% 
% B(1,1) = 0;
% B(2,1) = 0;
% B(3,1) = 207.1*dt;

A = diag([ 1 1 (1-18.69*dt)]);
A(1,2) = dt;
A(1,3) = 0;
A(2,1) = 0; 
A(2,3) = dt;
A(3,1) = -1001*dt;
A(3,2) = -148.2*dt;

B(1,1) = 0;
B(2,1) = 0;
B(3,1) = 522.5*dt;
%% discrete time model
% A = diag([(1+2.89*dt) 1 1 ]);
% A(1,2) = -1.394*dt;
% A(1,3) = 0.449*dt;
% A(2,1) = 2*dt; 
% A(2,3) = 0;
% A(3,1) = 0;
% A(3,2) = 1*dt;
% 
% B(1,1) = 0.007812*dt;
% B(2,1) = 0;
% B(3,1) = 0;
% 
% Ad = [ 2.89  -1.394  0.4491;
%        2       0       0;
%        0       1       0];
%  Bd = [0.007812 0 0];
%  Cd = [ 0.004301  0.008373  0.002038];
%  Dd = 0;
