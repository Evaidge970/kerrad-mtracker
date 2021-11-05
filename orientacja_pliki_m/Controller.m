function [u, qr, ur, qe] = Controller(q, t)

    [qr, ur] = TrajectoryGenerator(t);
    
    % Define control law
   
    zeta = 1.0;
    alfa = 50.0;
    
    R = [cos(qr(3)) -sin(qr(3)); sin(qr(3)) cos(qr(3))];
    
    qe = [ R'*[q(1)-qr(1); q(2)-qr(2)] ; q(3) - qr(3)];
      
    qe(3) = atan2(sin(qe(3)),cos(qe(3)));
    
    w0 = sqrt(ur(1)*ur(1)+ alfa*ur(2)*ur(2));
    
    k1= 2*zeta*w0;
    k3= k1;
    k2= (w0*w0 - ur(1)*ur(1))/ur(2);
    
    K = [0 k1 k2; k3 0 0];
    
    u = -K*qe + ur;
    
end

