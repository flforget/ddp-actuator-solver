function cost = ilqr_cost(u, x0, xf, Qf, Q, R, dt)

% newx = ilqr_cost(u, x0, xf, Qf, Q, R, dt)
%
% run the system under consideration forward using the command signal u
% and calculate the associated cost defined by xf, Qf, Q, R.
%
% given -> u the command signal with which to control the system
%	-> x0 the state of the system at some initial time
%	-> xf the desired final state of the system
%	-> Qf the quadratic final cost matrix
%	-> Q the quadratic integrated cost matrix on the state
%	-> R the quadratic integrated cost matrix on the command
%       -> dt the current time step used
%
% returns -> the associated cost.
%
% created by timothy lillicrap (tim@biomed.queensu.ca); december 1, 2005
% last edited by timothy lillicrap (tim@biomed.queensu.ca); december 1, 2005

x = ilqr_openloop(x0, u, dt);
N = length(x);

cost = (x(N,:) - xf) * Qf * (x(N,:) - xf)';

for i = 1:N-1
	cost = cost + (x(i,:)-xf) * Q * (x(i,:)-xf)' + u(i,:) * R * u(i,:)';
end;

cost = (1/2)*cost;
