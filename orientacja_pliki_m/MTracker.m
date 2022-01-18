%*************************************************************************
% Matlab interface for MTracker robot
% (c) KSIS, D. Pazderski 2015
%*************************************************************************

if (MTrackerDriver('open', [3, 921600]) == -1) % bylo 115200
    return;
end

Tf = 30;
Ts = 0.03;
d=0.1;
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

%zadane wspó³rzêdne

%czy zadac nowy punkt: 0 - pusta ramka, odczyt; 1 - wyslanie nowego rozkazu

%czy wyczyscic bufor kolejki: 0 - dodac rozkaz do kolejki; 1 - przerwac wykonanie zadania i wykonac od razu wyslany rozkaz

%tryb: 0 - pozycja; 1 - orientacja; 2 - pozycja z ustalon¹ prêdkoœci¹; 
% 3 -tryb none

while (tau < Tf)
    
    i = i+1;
    tau = toc;

    
    
    MTrackerDriver('highLevelControl',[0.0; 0.0; 0.0; 0;0;0]); %x, y, th, zadanie punktu (jesli 0 to wysylamy pusta ramke)

    if (done(2) == 0)
         MTrackerDriver('highLevelControl',[0.8; -1.1; 0.0; 1;0;2]);
         done(2) = 1;
    end
    if (done(1) == 0 && tau > 5 )
         MTrackerDriver('highLevelControl',[0.8; -1.1; -0.4; 1;1;1]);
         done(1) = 1;
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


   
 
 
%MTrackerDriver('highLevelControl',[0.0; 0.0; 4.0; 1])
MTrackerDriver('close');
q_z = ones(3,n);
for i=1:n
    q_z(1,i) = q(1,i)+d*cos(q(3,i));
    q_z(2,i) = q(2,i)+d*sin(q(3,i));
    q_z(3,i) = q(3,i);
end
    
    

% Adjust buffers
t = t(:, 1:i);
w = w(:, 1:i);
q = q(:, 1:i);




disp('Robot is stopped.');