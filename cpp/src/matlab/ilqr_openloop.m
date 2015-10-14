function trajectory = ilqr_openloop(x0, u, dt)

% trajectory = ilqr_openloop(x, u, dt)
%
% run the system forward in open loop from an initial state x0 using
% the control signal u.  note that length(trajectory) == length(u) + 1.
%
% given -> x0 the state of the system at some initial time
%       -> u the command signal into the system 
%       -> dt the current time step used
%
% returns -> the trajectory through state space taken by the system.
%
% created by timothy lillicrap (tim@biomed.queensu.ca); december 1, 2005
% last edited by timothy lillicrap (tim@biomed.queensu.ca); december 1, 2005

trajectory = zeros(length(u)+1, length(x0));

trajectory(1,:) = x0;

for i = 2:length(trajectory)
	trajectory(i,:) = ilqr_system(trajectory(i-1,:), u(i-1,:), dt);
end;

