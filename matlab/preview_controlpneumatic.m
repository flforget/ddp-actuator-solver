function [F] = preview_controlpneumatic(Tprev,EndTime)
   %% global Dtime
    
    G  = 9.81;
    %Zh = comz
    
    % integration scheme
    % dx/dt = Ac x + Bc u
    % y     = Cc x + Dc u
%     Ac = [0 1 0;
%           0 0 1;
%           0 0 0];
%     Bc = [0;0;1];
%     Cc = [1 0 -Zh/G];
%     Dc = [0];

    Dtime = 0.01;
%%   
    %Q = diag([1 1 1]);
    R = 1e-5;

%     A = diag([(-10.73) 0 0]);
%     A(1,2) = -10.53;
%     A(1,3) = -5.719;
%     A(2,1) = 8; 
%     A(2,3) = 0; 
%     A(3,1) = 0;
%     A(3,2) = 8;
% 
%     B(1,1) = 2;
%     B(2,1) = 0;
%     B(3,1) = 0;
% 
%     C = [0 0 1.6183];
%     D = 0;
     %% SIMPLE PENDULUM
%     g = 9.8;
%     m = 1.0;
%     l = 1.0;
%     mu = 0.01;
% 
%     A = diag([0 (-(mu/(m*l^2)))]);
%     A(1,2) = 1;
%     A(2,1) = -(g/l);
% 
%     B(1,1) = 0;
%     B(2,1) = (1/(m*l^2));
%     C = [1 0];
%     D = 0;
  %% Linear identified Pneumatic arm model
    g = 9.8;
    m = 1.0;
    l = 1.0;
    mu = 0.01;
   
    %%
    Ac = A;
    Bc = B;
    Cc =C;
    Dc =D;
%%
    % Compute optimal weight
    sys = c2d(ss(Ac,Bc,Cc,Dc),Dtime);
    % 
    Ax = [1 sys.C*sys.A;
          zeros(size(sys.A,1),1) sys.A];
    Bx = [sys.C*sys.B; sys.B];
    Cx = [1 zeros(1,size(sys.A,1))];

    % 
    Q = 1;
    R = 1E-5;
    [K, P, E] = dlqry(Ax,Bx,Cx,0,Q,R);
    Ks = K(1);
    Kx = K(2:end);

%% 
    R_BtPB = R + Bx'*P*Bx;    % scalar

    zeta = (Ax - Bx*K)';

    Nprev = round(Tprev/Dtime);
    time = zeros(1,Nprev);
    F = zeros(1,Nprev);
    Fsub = P * Cx'*Q;
    for n=1:Nprev
        F(n) = Bx' * Fsub/R_BtPB;
        Fsub = zeta * Fsub;
        time(n) = n*Dtime;
    end

    figure(1)
    plot(time, F);
   
    
    %%%%
    time = [0:Dtime:EndTime]';
    zmp = (0.15)*time;
%     StepL = 0.3;  % m
%     StepW = 0.1;  
%     Nsteps = 5;
%     Cycle = 0.8;  % s
% 
    tsize = length(time);
    zmpx2 = zeros(tsize-Nprev,1);
%     zmpy2 = zeros(tsize-Nprev,1);
    comx = zeros(tsize-Nprev,size(Ac,1));
%     comy = zeros(tsize-Nprev,size(Ac,1));
    sxzmp = 0.0;
%     syzmp = 0.0;
% 
    x = zeros(size(Ac,1),1);
%     y = zeros(size(Ac,1),1);
    for n=1:tsize-Nprev
        ux = -Kx * x + Ks * sxzmp + F*zmp(n:n+Nprev-1,1);
        u(n) = ux;
        %uy = -Kx * y + Ks * syzmp + F*zmp(n:n+Nprev-1,2);
        x = sys.A * x + sys.B * ux;
        xstate(:,n) = x;
        %y = sys.A * y + sys.B * uy;
        zmpx2(n) = (sys.C * x)';
        %zmpy2(n) = (sys.C * y)';
        sxzmp = sxzmp + (zmp(n,1)-zmpx2(n));
        %syzmp = syzmp + (zmp(n,2)-zmpy2(n));
        comx(n,:) = x';
        %comy(n,:) = y';
    end
% 
figure(2)
subplot(211), plot(time, zmp, 'g*');

hold on;
grid on;
subplot(211), plot(time(1:tsize-Nprev), zmpx2, 'r');
subplot(211), plot(time, x(1,:), 'b*');
subplot(212), plot(time(1:tsize-Nprev), u);
end
