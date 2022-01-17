figure;
plot(t, q(1,:), 'r'); hold on; grid on;
plot(t, q(2,:), 'g'); 
plot(t, q(3,:), 'b'); 
plot(t, q_z(1,:), 'r-');
plot(t, q_z(2,:), 'g-');
legend('x [m]', 'y [m]', '\theta [rad]', 'x_d[m]', 'y_d[m]');


