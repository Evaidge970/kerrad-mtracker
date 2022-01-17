figure();
subplot(1,3,1)

plot(q_z(1,:), q_z(2,:));
hold on;
plot(q(1,:), q(2,:));
hold on;
xlabel('x [m]'); ylabel('y [m]'); 
legend('q_z','q');

subplot(1,3,2)

plot(t, w(1,:), 'r'); hold on; grid on;
plot(t, w(2,:), 'g'); 
%plot(t, wd(1,:), 'b'); 
%plot(t, wd(2,:), 'y'); 
legend('w1', 'w2');

subplot(1,3,3)


plot(t, q(1,:), 'r'); hold on; grid on;
plot(t, q(2,:), 'g'); 
plot(t, q(3,:), 'b'); 
plot(t, q_z(1,:), 'r--');
plot(t, q_z(2,:), 'g--');
legend('x [m]', 'y [m]', '\theta [rad]', 'x_d[m]', 'y_d[m]');



