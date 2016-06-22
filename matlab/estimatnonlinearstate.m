%% 

dt = 0.005;


joint_state_cur = [0 0 0 4]';
u = [0.0*1e5 4*1e5]';
l =1;
% for i = 0:0.005:10
%     u = [2*1e5 2*1e5]';
%     [jointstate_deriv,Jx, Ju] = jointnonlineardynamics(joint_state_cur, dt,u);
%     %[jointstate_deriv] = pendulumdynamics(joint_state_cur, dt,u);
%     Jx
%     joint_state_cur =  jointstate_deriv;
%     joint_pos(l,1:4) = joint_state_cur(1:4,1);
% %     force1(l) = F(1);
% %     force2(l) = F(2);
%     tim(l) = i;
%     l = l+1;
% end
ode45('jointdynamics4bkeuler', [0 10],joint_state_cur);
ode15s('jointdynamics4bkeuler', [0 10],joint_state_cur);
% figure(1)
% subplot(221), plot(tim, joint_pos(:,1))
% subplot(222), plot(tim, joint_pos(:,2))
% subplot(223), plot(tim, joint_pos(:,3))
% subplot(224), plot(tim, joint_pos(:,4)) 


