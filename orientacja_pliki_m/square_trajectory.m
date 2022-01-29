function [q,q_z,t,w,Tf,n] = square_trajectory(a,mode,k,V,e,d);
% wywo?anie funkcji powoduje narysowanie kwadratu o zadanej g?ugo?ci boku przez punkt z
% je?eli który? z parametrów ma zosta? domy?lny, nale?y wpisa? 0
if (MTrackerDriver('open', [4, 921600]) == -1) % bylo 115200
    return;
end

if k == 0
    k = 0.2;
end
if V == 0
    V = 0.1;
end
if e == 0
    e = 0.01;
end
if d == 0
    d = 0.1;
end

if mode == "const_velocity"
    Tf = 1.1*4*a/V;
elseif mode == "proportional"
    Tf = -4*1.2*(log(e / a))/k;
end
Ts = 0.03;
n = floor(Tf/Ts)+1;

% Initialization of buffers to store data
t = zeros(1, n);
dataReady = zeros(1, n);
w = zeros(2, n);
q = zeros(3, n);

% Initial localization
q_i = [0; 0; 0];
w_i = [0;0];

i = 0; tau = 0;
tic;

MTrackerDriver('setOdometry', q_i); 
wait(tau, 0.1);
data = MTrackerDriver('read');

done = zeros(10);


tic;
disp('Robot is started.');
%Format komendy jest zamieszczony w pliku MTracker.m
while (tau < Tf)
    
    i = i+1;
    tau = toc;
    if (done(1) == 0 )
    MTrackerDriver('highLevelControl',[k; d; e; 0;1;0; 2]); %x, y, th, zadanie punktu (jesli 0 to wysylamy pusta ramke)
    done(1) = 1;
    elseif (done(2) == 0 )
    MTrackerDriver('highLevelControl',[V; 0.5; e; 0;1;0; 3]);
    done(2) = 1;
    else
    MTrackerDriver('highLevelControl',[0.0; 0.0; 0.0; 0;0;0;0]); %x, y, th, zadanie punktu (jesli 0 to wysylamy pusta ramke)
    end
if (mode == "proportional")
    if (  done(3) == 0)
        MTrackerDriver('highLevelControl',[a+d; 0.0; 0.0; 1;0;0;0]);
        done(3) = 1;
    end
    if (  done(4) == 0)
        MTrackerDriver('highLevelControl',[a+d; a; 0.0; 1;0;0;0]);
        done(4) = 1;
    end
 
    if (   done(5) == 0)
        MTrackerDriver('highLevelControl',[d; a; 0.0; 1;0;0;0]);
        done(5) = 1;
    end

    if ( done(6) == 0)
        MTrackerDriver('highLevelControl',[d; 0; 0.0; 1;0;0;0]);
        done(6) = 1;
    end
 elseif mode == "const_velocity"
    if (  done(3) == 0)
        MTrackerDriver('highLevelControl',[a+d; 0.0; 0.0; 1;0;2;0]);
        done(3) = 1;
    end

    if (  done(4) == 0)
        MTrackerDriver('highLevelControl',[a+d; a; 0.0; 1;0;2;0]);
        done(4) = 1;
    end
 
    if (   done(5) == 0)
        MTrackerDriver('highLevelControl',[d; a; 0.0; 1;0;2;0]);
        done(5) = 1;
    end

    if ( done(6) == 0)
        MTrackerDriver('highLevelControl',[d; 0; 0.0; 1;0;2;0]);
        done(6) = 1;
    end
 end
    
    wait(tau, Ts);
    data = MTrackerDriver('read');
    
    % Check if new data is available
    if (data(1) == 1)
        q_i = data(2:4)';
        w_i = data(5:6)';
    end

    
    dataReady(:,i) = data(1);
    q(:,i) = q_i;
    w(:,i) = w_i; 
    t(i) = tau;  

end

MTrackerDriver('close');
q_z = ones(3,n);
for l=1:n
    q_z(1,l) = q(1,l)+d*cos(q(3,l));
    q_z(2,l) = q(2,l)+d*sin(q(3,l));
    q_z(3,l) = q(3,l);
end

t = t(:, 1:i);
w = w(:, 1:i);
q = q(:, 1:i);
q_z = q_z(:, 1:i);



disp('Robot is stopped.');
end