plot(q_z(1,1:n-5), q_z(2,1:n-5));
hold on;
grid on;
plot(q(1,1:n-5), q(2,1:n-5));
%plot(0.4,0.4,'x');
%plot(-0.2,-0.2,'x');
hold on;
xlabel('x [m]'); ylabel('y [m]'); 
legend('q_z','q');
title({'trajektoria ruchu','robota na p≥aszczyünie'})
