plot(t, w(1,:), 'r'); hold on; grid on;
plot(t, w(2,:), 'g'); 
%plot(t, wd(1,:), 'b'); 
%plot(t, wd(2,:), 'y'); 
legend('\omega_R', '\omega_L');
xlabel('t [s]');
ylabel('\omega [rad/s]');
title({'Prêdkoœci obrotowe',' kó³ robota'})