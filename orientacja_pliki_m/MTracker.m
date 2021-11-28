%*************************************************************************
% Matlab interface for MTracker robot
% (c) KSIS, D. Pazderski 2015
%*************************************************************************

if (MTrackerDriver('open', [3, 921600]) == -1) % bylo 115200
    return;
end

Tf = 20;
Ts = 0.03;

n = floor(Tf/Ts)+1;

% Initialization of buffers to store data
t = zeros(1, n);
dataReady = zeros(1, n);
w = zeros(2, n);
wd = zeros(2, n);
ud = zeros(2, n);
ur = zeros(2, n);
q = zeros(3, n);
qr = zeros(3, n);

% Define user data
qe = zeros(3,n);


% Initial localization
q_i = [0; 0; 0];
w_i = [0;0];

i = 0; tau = 0;
tic;

MTrackerDriver('setOdometry', q_i); 
wait(tau, 0.1);
data = MTrackerDriver('read');


tic;
disp('Robot is started.');
% Main control loop
while (tau < Tf)
    
    i = i+1;
    tau = toc;

    % Compute control law
    [ud_i, qr_i, ur_i, qe_i] = Controller(q_i, tau);
    wd_i = ComputeWheelsVelocities(ud_i);
            
    % Communication with the robot
    % MTrackerDriver('sendVelocity', wd_i); 
    if(tau < 1)
    MTrackerDriver('highLevelControl',[0.0; 0.0; 3.0; 1]); //x, y, th, zadanie punktu (jesli 0 to wysylamy pusta ramke)
    elseif (tau <10)
    MTrackerDriver('highLevelControl',[0.0; 0.0; 1.0; 1]);
    else
    MTrackerDriver('highLevelControl',[0.0; 0.0; 0.5; 1]);
    end
    %if(tau < 5)
    %MTrackerDriver('highLevelControl',[0.0; 0.0; 0.0; 1]);
    %elseif (tau < 10)
    %MTrackerDriver('highLevelControl',[0.0; 0.0; 4.0; 1]);
   % MTrackerDriver('highLevelControl',[0.0; 0.0; 4.0; 1]);
    %end
    wait(tau, Ts);
    data = MTrackerDriver('read');
    
    % Check if new data is available
    if (data(1) == 1)
        q_i = data(2:4)';
        w_i = data(5:6)';
    end
    
    dataReady(:,i) = data(1);
    q(:,i) = q_i;
    qr(:,i) = qr_i;
    w(:,i) = w_i; 
    wd(:,i) = wd_i;
    ud(:,i) = ud_i;
    ur(:,i) = ur_i;
    t(i) = tau;  
    
    qe(:,i) = qe_i;
end    
%MTrackerDriver('highLevelControl',[0.0; 0.0; 4.0; 1])
MTrackerDriver('close');

% Adjust buffers
t = t(:, 1:i);
w = w(:, 1:i);
wd = wd(:, 1:i);
ud = ud(:, 1:i);
ur = ur(:, 1:i);
qr = qr(:, 1:i);
q = q(:, 1:i);
qe = qe(:, 1:i);

disp('Robot is stopped.');