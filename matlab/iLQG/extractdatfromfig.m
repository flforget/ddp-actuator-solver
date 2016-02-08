f =openfig('ramptrackMPC.fig') %open your fig file, data is the name I gave to my file
H = findobj(f,'type', 'line')

%D=get(gca,'Children'); %get the handle of the line object
XData=get(H,'XData'); %get the x data
YData=get(H,'YData'); %get the y data
figure(2)

subplot(211), plot((1:800)*5e-3, (180/pi)*YData{3,1}, 'Linewidth', 2.0);
hold on;
grid on;
subplot(211), plot((1:800)*5e-3, (180/pi)*YData{2,1}, 'r', 'Linewidth', 2.0);

ylabel('Position (Degrees)')
legend('Model response', 'Reference');
title('MPC based Trajectory tracking', 'FontSize',30)

subplot(212), plot((1:800)*5e-3, 1e-5*YData{1,1}, 'Linewidth', 2.0);
grid on;
ylabel('Control input (Bar)')
xlabel('Time (s)')