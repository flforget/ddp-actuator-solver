function newx = ilqr_system(x, u, dt)

% newx = ilqr_system(x, u, dt)
%
% run the system under consideration forward and return the new state.
%
% given -> x the state of the system at some time
%       -> u the command into the system at some time
%       -> dt the current time step used
%
% returns -> newx the new state of the system
%
% created by timothy lillicrap (tim@biomed.queensu.ca); december 1, 2005
% last edited by timothy lillicrap (tim@biomed.queensu.ca); december 1, 2005

newx = zeros(size(x));
[A B] = ilqr_linearization(x,u,dt);
newx = (A*x' + B*u')';

