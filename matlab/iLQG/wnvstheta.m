%% Volume vs Natural frequency of a muscle
function wnvstheta

theta = 0:0.01:3;

%joint1_R = p1*x(1,1) + p2;
joint1_lo = 0.23;
joint1_alphaob = 20.0*pi/180;
joint1_alphaot = 20.0*pi/180;
joint1_k = 1.1;
joint1_ro = 0.012;
joint1_R = 0.0095;
fv1 = 3.0;
[wnb1,Vb1, Vt1] = omegacal(theta,joint1_lo,joint1_alphaob,joint1_k,joint1_ro,joint1_R);

%% Parameters for the muscles at Joint 2
joint2_lo = 0.185;
joint2_alphaob = 23.0*pi/180;
joint2_alphaot = 23.0*pi/180;
joint2_k = 1.25;
joint2_ro = 0.0085;
joint2_R = 0.015;
fv2 = 0.25;
[wnb2,Vb2, Vt2] = omegacal(theta,joint2_lo,joint2_alphaob,joint2_k,joint2_ro,joint2_R);


%subplot(211), plot(theta,wnb1, 'Linewidth', 2.0);
hold on; grid on;
subplot(211), plot(theta,wnb2, 'b', 'Linewidth', 2.0);
xlabel('Angular Position (rad)')
ylabel('wn')
legend('agostic muscle at joint 1 (larger)','agostic muscle at joint 2 (smaller)' )

%subplot(212), plot(theta,Vb1, 'Linewidth', 2.0);
hold on; grid on;
subplot(212), plot(theta,Vb2, 'b', 'Linewidth', 2.0);
xlabel('Angular Position (rad)')
ylabel('Volume')
legend('agostic muscle at joint 1 (larger)','agostic muscle at joint 2 (smaller)' )
function [wnb,Vb,Vt] = omegacal(theta,lo,alphaob,k,ro,R)
%theta = 0;
a_biceps = 3/(tan(alphaob))^2;
b_biceps = 1/(sin(alphaob))^2;
emax_biceps = (1/k)*(1 - sqrt(b_biceps/a_biceps));
alphaot = alphaob;
a_triceps = 3/(tan(alphaot))^2;
b_triceps = 1/(sin(alphaot))^2;
emax_triceps = (1/k)*(1 - sqrt(b_triceps/a_triceps));

lb = lo - R.*theta;
epsb = (1-(lb./lo));
lt = lo*(1-emax_triceps) + R.*theta;
epst = (1-(lt./lo));

%% Parameters of Joint
% m = 2.6;
% link_l = 0.32;
% g =9.81;
% I = m*(link_l^2)/3;
% fv = 0.25;
%% Volume calcuation
% biceps agonistic muscle
lb = lo - R.*theta;
csb2 = (cos(alphaob))^2;
epsb2 = (1-(lb./lo)).^2;
termb1 = (1 - csb2.*epsb2);
element1 = lb.*(pi*ro^2/((sin(alphaob))^2));
element2 = termb1.*1e6;
Vb = tt(element1,element2);
%Vb = 1630;
wnb = 2*pi*380.*(1./Vb);

% triceps antagonistic muscle 
lt = lo*(1-emax_triceps) + R.*theta; 
cst2 = (cos(alphaot))^2;
epst2 = (1-(lt./lo)).^2;
termt1 = (1 - cst2.*epst2);
elementt1 = lt.*(pi*ro^2/((sin(alphaot))^2));
elementt2 = termt1.*1e6;
Vt = tt(elementt1,elementt2);

%Vt = 1e6*(pi*lt*ro^2/((sin(alphaot))^2))*termt1
%Vt = 1630;
wnt = 2*pi*380*(1./Vt);



% utility functions, singleton-expanded addition and multiplication
function c=pp(a,b)
c = bsxfun(@plus,a,b);

function c=tt(a,b)
c = bsxfun(@times,a,b);

function c=mm(a,b)
c = bsxfun(@minus,a,b);