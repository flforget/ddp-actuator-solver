%% PLOT
dt = 5e-3; % dt for dynamics
ToT = 10;       %(in seconds)
N_tot = ToT/dt;
figure(1)
%% Position plots
% Joint 1
subplot(321), plot((1:N_tot-200)*dt, (180/pi)*simout_mb.Data(1:N_tot-200,1), 'k--', 'Linewidth', 2.0);
title('Joint 1')
hold on;
grid on;
subplot(321), plot((1:N_tot-200)*dt, xtrajout_pm34(1,1:N_tot-200), 'b', 'Linewidth', 2.0);

subplot(321), plot((1:N_tot-200)*dt, xtrajout_pm45(1,1:N_tot-200), 'c--', 'Linewidth', 2.0);
subplot(321), plot((1:N_tot-200)*dt, (180/pi)*0.5, 'r', 'Linewidth', 2.0);

subplot(321), plot((1:N_tot-200)*dt, xtrajout_opt(1,1:N_tot-200), 'm--', 'Linewidth', 2.0);
ylabel('Position (Degrees)')
legend('Feedforward PI ', 'Optimal position control at preset Pm = 3 bar ', 'Optimal position control at preset Pm = 4 bar', 'Reference Position', 'Optimal Position and stiffness control');

% Joint 2
subplot(322), plot((1:N_tot-200)*dt, (180/pi)*simout_mb.Data(1:N_tot-200,2), 'k--', 'Linewidth', 2.0);

title('Joint 2')
hold on;
grid on;
subplot(322), plot((1:N_tot-200)*dt, xtrajout_pm34(2,1:N_tot-200), 'b', 'Linewidth', 2.0);

subplot(322), plot((1:N_tot-200)*dt, xtrajout_pm45(2,1:N_tot-200), 'c--', 'Linewidth', 2.0);
subplot(322), plot((1:N_tot-200)*dt, (180/pi)*1, 'r', 'Linewidth', 2.0);
subplot(322), plot((1:N_tot-200)*dt, xtrajout_opt(2,1:N_tot-200), 'm--', 'Linewidth', 2.0);
legend('Feedforward PI ', 'Optimal position control at preset Pm = 3 bar ', 'Optimal position control at preset Pm = 4 bar', 'Reference Position', 'Optimal Position and stiffness control');


%% Stiffness plots
% Joint 1
subplot(323), plot((1:N_tot-200)*dt, simout_mb.Data(1:N_tot-200,5), 'k--', 'Linewidth', 2.0);
hold on;
grid on;
subplot(323), plot((1:N_tot-200)*dt, s_pm34(1,1:N_tot-200), 'b', 'Linewidth', 2.0);

subplot(323), plot((1:N_tot-200)*dt, s_pm45(1,1:N_tot-200), 'c--', 'Linewidth', 2.0);
subplot(323), plot((1:N_tot-200)*dt, s_opt(1,1:N_tot-200), 'm--', 'Linewidth', 2.0);

ylabel('Stiffness (N-m/rad)')
xlabel('Time (seconds)')
legend('Feedforward PI ', 'Optimal position control at preset Pm = 3 bar ', 'Optimal position control at preset Pm = 4 bar', 'Optimal Position and stiffness control');


% Joint 2
subplot(324), plot((1:N_tot-200)*dt, simout_mb.Data(1:N_tot-200,6), 'k--', 'Linewidth', 2.0);
hold on;
grid on;
subplot(324), plot((1:N_tot-200)*dt, s_pm34(2,1:N_tot-200), 'b', 'Linewidth', 2.0);
subplot(324), plot((1:N_tot-200)*dt, s_pm45(2,1:N_tot-200), 'c--', 'Linewidth', 2.0);
subplot(324), plot((1:N_tot-200)*dt, s_opt(2,1:N_tot-200), 'm--', 'Linewidth', 2.0);

xlabel('Time (seconds)')
legend('Feedforward PI ', 'Optimal position control at preset Pm = 3 bar ', 'Optimal position control at preset Pm = 4 bar', 'Optimal Position and stiffness control');

%% Pressure plots
% Joint 1
subplot(325), plot((1:N_tot-200)*dt, (pi/180)*xtrajout_pm34(5,1:N_tot-200), 'b*-', 'Linewidth', 2.0);
title('Joint 1')
hold on;
grid on;
subplot(325), plot((1:N_tot-200)*dt, 3 - (pi/180)*xtrajout_pm34(5,1:N_tot-200), 'b--', 'Linewidth', 2.0);

subplot(325), plot((1:N_tot-200)*dt, (pi/180)*xtrajout_pm45(5,1:N_tot-200), 'c', 'Linewidth', 2.0);
subplot(325), plot((1:N_tot-200)*dt, 4 - (pi/180)*xtrajout_pm45(5,1:N_tot-200), 'c--', 'Linewidth', 2.0);


subplot(325), plot((1:N_tot-200)*dt, (pi/180)*xtrajout_opt(5,1:N_tot-200), 'm', 'Linewidth', 2.0);
subplot(325), plot((1:N_tot-200)*dt, (pi/180)*xtrajout_opt(6,1:N_tot-200), 'm--', 'Linewidth', 2.0);
ylabel('Pressure agonist  (Bar)')
legend('Pm = 3 bar agonist muscle ', 'Pm = 3 bar antagonist muscle ', 'Pm = 4 bar agonist muscle', 'Pm = 4 bar antagonist muscle ','Optimal Pm agonist muscle', 'Optimal Pm antagonist muscle ');
%legend( 'Optimal position control at preset Pm = 3 bar ', 'Optimal position control at preset Pm = 4 bar', 'Optimal Position and stiffness control');

% Joint 2
subplot(326), plot((1:N_tot-200)*dt, (pi/180)*xtrajout_pm34(6,1:N_tot-200), 'b*-', 'Linewidth', 2.0);
title('Joint 2')
hold on;
grid on;
subplot(326), plot((1:N_tot-200)*dt, 4 - (pi/180)*xtrajout_pm34(6,1:N_tot-200), 'b--', 'Linewidth', 2.0);

subplot(326), plot((1:N_tot-200)*dt, (pi/180)*xtrajout_pm45(6,1:N_tot-200), 'c', 'Linewidth', 2.0);
subplot(326), plot((1:N_tot-200)*dt, 5 - (pi/180)*xtrajout_pm45(6,1:N_tot-200), 'c--', 'Linewidth', 2.0);


subplot(326), plot((1:N_tot-200)*dt, (pi/180)*xtrajout_opt(9,1:N_tot-200), 'm', 'Linewidth', 2.0);
subplot(326), plot((1:N_tot-200)*dt, (pi/180)*xtrajout_opt(10,1:N_tot-200), 'm--', 'Linewidth', 2.0);
ylabel('Pressure agonist  (Bar)')
legend('Pm = 3 bar agonist muscle ', 'Pm = 3 bar antagonist muscle ', 'Pm = 4 bar agonist muscle', 'Pm = 4 bar antagonist muscle ','Optimal Pm agonist muscle', 'Optimal Pm antagonist muscle ');

