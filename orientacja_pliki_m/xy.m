plot(q_z(1,1:n-5), q_z(2,1:n-5));
hold on;
grid on;
plot(q(1,1:n-5), q(2,1:n-5));
%plot(0.6, 0.7,'x');
%plot(1.2,1.7,'x');
%plot(-1.2,1.4,'x');
hold on;
xlabel('x [m]'); ylabel('y [m]'); 
legend('z','p','(0.6, 0.7) [m]');
%title({'trajektoria ruchu','robota na p≥aszczyünie'})
