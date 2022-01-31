plot(q_z(1,1:n-5), q_z(2,1:n-5));
hold on;
grid on;
plot(q(1,1:n-5), q(2,1:n-5));
plot(0.5, 0.8,'go');
%plot(0.7,0.9,'x');
%plot(-0.3,0.6,'x');
hold on;
xlabel('x [m]'); ylabel('y [m]'); 
legend('z','p','(0.5, 0.8) [m]');
%title({'trajektoria ruchu','robota na p≥aszczyünie'})
