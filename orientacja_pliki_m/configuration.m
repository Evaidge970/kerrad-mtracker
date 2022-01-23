plot(t(1:n-5), q(1,1:n-5), 'r'); hold on; grid on;
plot(t(1:n-5), q(2,1:n-5), 'g'); 
plot(t(1:n-5), q(3,1:n-5), 'b'); 
plot(t(1:n-5), q_z(1,1:n-5), 'r--');
plot(t(1:n-5), q_z(2,1:n-5), 'g--');
legend('x [m]', 'y [m]', '\theta [rad]', 'x_z[m]', 'y_z[m]');
title({'Przebieg zmiennych','konfiguracyjnych robota'})
xlabel('t[s]');