
%% Linear model

% u = zeros(50,2);
% u = ilqr(u, [0 0 0 0], [1 4 0 0], diag([1 1 1 1]),zeros(4),0.001*eye(2), 0.05, 20, 0); %ilqr(u, x0, xf, Qf, Q, R, dt, iterations, ethreshold)
% x = ilqr_openloop([0 0 0 0], u, 0.05); 

%% Simple pendulum
% u = zeros(120, 1);
% u = ilqr(u, [pi, 0], [0 0], diag([1 1]), zeros(2), 0.00001*eye(1), 0.005, 20, 0);
% x = ilqr_openloop([pi 0], u, 0.005); plot(x(:,1), x(:,2));

% N = 120;
% u = zeros(N, 1);
% 
% %ilqr_system(x0,u, dt);
% x0 = [pi/2 0];
% xf = [0 0];
% Qf = diag([1 1]);
% Q = zeros(2);
% R = 0.00001*eye(1);
% dt = 0.005;
% iterations = 20;
% ethreshold = 0.0000001;
% Timeofsim = 1.0;

%% Pneumaticarm Model
%N = 800;
%u = zeros(N, 1);

%ilqr_system(x0,u, dt);
% x0 = [0 0];
% xf = [1 0];
% Qf = diag([1 1 ]);
% Q = zeros(2);
% R = 0.00001*eye(1);
% dt = 0.005;
% iterations = 20;
% ethreshold = 0.0000000001;
% Timeofsim = 4;
% N = Timeofsim/(10*dt);
% u = zeros(N, 1);


%% Bertrand Model
% g = 9.8;
% m = 2.75;
% link_l = 0.32;
% 
% I = 0.25*m*(link_l^2)/3;
% K1 = 2.1794;
% K2 = 1.2698;
% Pm = 2.5 ;
% 
% Tmax = 5*K1;
% fk = 0.1*Tmax;
% fs = fk/10;
% fv = fk;
% %Linear around (0,0)
% A = [0 1; (-2*K2*Pm/I - m*g*link_l/I)  -fv/I];
% B = [0 ; 2*K1/I];
% C = [1 0 ];
% D = 0;
% ss_linearbertrand = ss(A,B,C,D);

%% Identified Lienar Model
% N = 81;
% u = zeros(N, 1);
% 
% %ilqr_system(x0,u, dt);
x0 = [0 0 0 ];
xf = [1 0 0];
Qf = diag([1 1 1]);
Q = 1e-4*Qf; %zeros(3);
R = 0.00001*eye(1);
dt = 0.005;
iterations = 20;
ethreshold = 0.0000000001;
Timeofsim = 4;
N = Timeofsim/(10*dt);
u = zeros(N, 1);

%% 
xcur = x0;

%% One loop
% u = ilqr(u, xcur, xf, Qf, Q, R, dt, iterations, ethreshold);
% x = ilqr_openloop(x0, u, dt); 
% figure(1)
% subplot(211), plot(x(:,1))
% hold on;
% grid on;
% xlabel('Time');
% ylabel('Angular postion of elbow joint (rad)')
% 
% subplot(212), plot(u(:,1))
% grid on;
% xlabel('Time');
% ylabel('Pressure variation (bar)')
% set(gca,'FontSize',30,'fontWeight','bold')
%% TIme evolution
xtime = zeros(Timeofsim/(dt) +1,3);
utime = zeros(Timeofsim/(dt),1);
lp =0;
lpn =1;
int=1;
mstep =1;
finiter = Timeofsim/(dt);

fin = Timeofsim/(dt*N);
mstep = fin;
xtime(1,:) = xcur;
ucur = u;
%[waypoint, T,Xref] = lsim(ss_linearbertrand, u_ref, time_ref);
for i = 1:1:finiter
   % xf(:,1) = waypoint(lp) 
   
    u = ilqr(ucur, xcur, xf, Qf, Q, R, dt, iterations, ethreshold);
    xtraj = ilqr_openloop(xcur, u, dt) ;%+ 0.001*rand(1);
    xcur = xtraj(2,:);
    xtime(i+1,:) = xcur;
    utime(i+1) = u(1);
    %u =  u(1:801-i);
    lp = lp+1;
    
    if(lp >= fin)
        ucur = zeros(N-lpn,1);
        ucur = u(1:N+1-lpn,1);
        
        lpn = lpn+1;   
        lp=0;
    end
    
    
end

figure(1)
subplot(211), plot(xtime(:,1))
hold on;
grid on;
xlabel('Time');
ylabel('Angular postion of elbow joint (rad)')

subplot(212), plot(utime(:,1))
grid on;
xlabel('Time');
ylabel('Pressure variation (bar)')
set(gca,'FontSize',30,'fontWeight','bold')

set(findall(gcf,'type','text'),'FontSize',30,'fontWeight','bold')
figure(2)
plot(xtime(:,1), xtime(:,2))
grid on;

xlabel('Angular postion of elbow joint (rad)');
ylabel('Angular speed of elbow joint (rad)');

%% For Preview gain
% v1 = ones(N, 1);
% v2 = ones(N, 1);
% v3 = ones(N, 1);
% 
% v1(:) = K(1,1,:);
% v2(:) = K(1,2,:);
% v1(:) = K(1,3,:);
% 
% figure(3)
% plot(v1, 'r')
% hold on
% plot(v2, 'g')
% plot(v3, 'b')
% ss_sys = ss(tf1);
% u = u'
% utraj = zeros(length(u)+1,1);
% utraj(1) = 0;
% for lp =3:1:N+2
%     utraj(lp-1) = u(lp-2);
% end
% utraj(1:N) = u(1:N) ;
% utraj(N+1:2*N+1) = u(N);
% utraj;
% Tfinal = dt*(2*N);
% T = 0:dt:Tfinal;
%u = 1+zeros(size(T));
%% Discrete time model
% dsys = c2d(tf1, 0.01, 'zoh');
% ss_dsys = ss(dsys)
% Ad = [ 2.89  -1.394  0.4491;
%        2       0       0;
%        0       1       0];
%  Bd = [0.007812 0 0]';
%  Cd = [ 0.004301  0.008373  0.002038];
%  Dd = 0;
%  
% xtrajd = dlsim(Ad, Bd, Cd, Dd, utraj)'
% x0 = [0, 0, 0]';
% 
% xtraj = lsim(ss_sys, utraj, T, x0)'
% % plot(xtrajd, 'r')
% plot(xtraj, 'g')