plot(t(1:n-5), w(1,1:n-5), 'r'); hold on; grid on;
plot(t(1:n-5), w(2,1:n-5), 'g'); 
%plot(t(1:n-5), wd(1,:), 'b'); 
%plot(t, wd(2,:), 'y'); 
legend('\omega_R', '\omega_L');
xlabel('t [s]');
ylabel('\omega [rad/s]');
%title({'Prêdkoœci obrotowe',' kó³ robota'})