figure;

plot(q_z(1,:), q_z(2,:));
hold on;
plot(q(1,:), q(2,:));
hold on;
xlabel('x [m]'); ylabel('y [m]'); 
legend('q_z','q');

