figure();
subplot(1,3,1)

plot(q_z(1,1:n-5), q_z(2,1:n-5));
hold on;
plot(q(1,1:n-5), q(2,1:n-5));
hold on;
xlabel('x [m]'); ylabel('y [m]'); 
legend('q_z','q');

subplot(1,3,2)

plot(t, w(1,:), 'r'); hold on; grid on;
plot(t, w(2,:), 'g'); 
%plot(t, wd(1,:), 'b'); 
%plot(t, wd(2,:), 'y'); 
legend('w1', 'w2');
xlabel('t [s]');

subplot(1,3,3)


plot(t(1:n-5), q(1,1:n-5), 'r'); hold on; grid on;
plot(t(1:n-5), q(2,1:n-5), 'g'); 
plot(t(1:n-5), q(3,1:n-5), 'b'); 
plot(t(1:n-5), q_z(1,1:n-5), 'r--');
plot(t(1:n-5), q_z(2,1:n-5), 'g--');
legend('x [m]', 'y [m]', '\theta [rad]', 'x_z[m]', 'y_z[m]');
xlabel('t[s]');



