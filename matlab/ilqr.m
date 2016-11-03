function newu = ilqr(u, x0, xf, Qf, Q, R, dt, iterations, ethreshold)

% du = ilqr(u, x0, xf, Qf, Q, R, dt, iterations, ethreshold)
%
% run the ilqr algorithm using some initial state, nominal comman signal,
% and cost function.  iterate until the maximum iterations are reached
% or until the change in the command signal goes below some threshold.
%
% given -> u
%	-> x0 the state of the system at some initial time
%       -> xf the requested goal state of the system
%       -> Qf the quadratic final cost matrix
%	-> Q the quadratic integrated cost matrix on the state
%	-> R the quadratic integrated cost matrix on the command
%	-> dt the current time step used
%	-> iterations the maximum number of iterations to run for
%	-> ethreshold stop when the change in u goes below this scalar
%
% returns -> newu the new nominal trajectory given by the algorithm
%
% created by timothy lillicrap (tim@biomed.queensu.ca); december 1, 2005
% last edited by timothy lillicrap (tim@biomed.queensu.ca); december 1, 2005

currentu = u;

oldcost = ilqr_cost(u, x0, xf, Qf, Q, R, dt);

for i = 1:iterations
	
	du = ilqr_iteration(currentu, x0, xf, Qf, Q, R, dt);	
	currentu = currentu + 0.25 * du;
	
	newcost = ilqr_cost(currentu, x0, xf, Qf, Q, R, dt);
	
	if(abs(newcost - oldcost) < ethreshold) 
		break;
	end;
		
	oldcost = newcost;	

end;

newu = currentu;
