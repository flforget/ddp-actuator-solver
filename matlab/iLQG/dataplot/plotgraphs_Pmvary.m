%% PLOT
dt = 5e-3; % dt for dynamics
ToT = 10;       %(in seconds)
N_tot = ToT/dt;
figure(1)
% Position plots
subplot(221), plot((1:N_tot-200)*dt, xtrajout_pm34(1,1:N_tot-200), 'b', 'Linewidth', 2.0);
title('Joint 1')
hold on;
grid on;
subplot(221), plot((1:N_tot-200)*dt, xtrajout_pm45(1,1:N_tot-200), 'c--', 'Linewidth', 2.0);
subplot(221), plot((1:N_tot-200)*dt, (180/pi)*0.5, 'r', 'Linewidth', 2.0);
ylabel('Position (Degrees)')
legend('Pm = 3 bar ', 'Pm = 4 bar');

subplot(222), plot((1:N_tot-200)*dt, xtrajout_pm34(2,1:N_tot-200), 'b', 'Linewidth', 2.0);
title('Joint 2')
hold on;
grid on;
subplot(222), plot((1:N_tot-200)*dt, xtrajout_pm45(2,1:N_tot-200), 'c--', 'Linewidth', 2.0);
subplot(222), plot((1:N_tot-200)*dt, (180/pi)*1, 'r', 'Linewidth', 2.0);
legend('Pm = 4 bar ', 'pm = 5 bar');

% Stiffness plots
subplot(223), plot((1:N_tot-200)*dt, s_pm34(1,1:N_tot-200), 'b', 'Linewidth', 2.0);
title('Joint 1')
hold on;
grid on;
subplot(223), plot((1:N_tot-200)*dt, s_pm45(1,1:N_tot-200), 'b--', 'Linewidth', 2.0);
ylabel('Stiffness (N/rad)')
xlabel('Time (seconds)')
legend('Pm = 3 bar ', 'Pm = 4 bar');

subplot(224), plot((1:N_tot-200)*dt, s_pm34(2,1:N_tot-200), 'b', 'Linewidth', 2.0);
title('Joint 2')
hold on;
grid on;
subplot(224), plot((1:N_tot-200)*dt, s_pm45(2,1:N_tot-200), 'b--', 'Linewidth', 2.0);
legend('Pm = 4 bar ', 'Pm = 5 bar');
xlabel('Time (seconds)')
% % subplot(211), plot((1:N_tot)*dt, (180/pi)*testgoal1(1,:),'r', 'Linewidth', 2.0);
% subplot(211), plot((1:N_tot-100)*dt, xtrajout(1,1:N_tot-100), 'k', 'Linewidth', 2.0);
% subplot(211), plot((1:N_tot-100)*dt, (180/pi)*1, 'cy','Linewidth', 2.0);
% % subplot(211), plot((1:N_tot)*dt, (180/pi)*testgoal2(1,:),'m', 'Linewidth', 2.0);
% ylabel('Position (Degrees)')
% legend('Joint 1 ', 'Joint 2');
% title('DDP optimal weight lifting', 'FontSize',30)
% 
% subplot(212), plot((1:N_tot)*dt, uhat(1,1:N_tot), 'Linewidth', 2.0);
% hold on;
% subplot(212), plot((1:N_tot)*dt, uhat(3,1:N_tot), 'c','Linewidth', 2.0);
% ylabel('Control Input (Bar)');
% xlabel('Time (s)');
% legend('Input 1 ', 'Input 2');
% 
% %% PLOT of stiffness
% figure(2)
% subplot(211), plot((1:N_tot)*dt, Tnet(1,1:N_tot),  'Linewidth', 2.0);
% hold on;
% grid on;
% subplot(211), plot((1:N_tot)*dt, Tnet(2,1:N_tot), 'r', 'Linewidth', 2.0);
% ylabel('Net Torque (Degrees)')
% legend('Joint 1 ', 'Joint 2');
% title('Net Joint Torues: 0ptimal weight lifting', 'FontSize',30)
% 
% subplot(212), plot((1:N_tot)*dt, s(1,1:N_tot), 'Linewidth', 2.0);
% hold on;
% subplot(212), plot((1:N_tot)*dt, s(2,1:N_tot), 'r','Linewidth', 2.0);
% ylabel('Stifness');
% xlabel('Time (s)');
% legend('Joint 1 ', 'Joint 2');