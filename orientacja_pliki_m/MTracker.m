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
q_i = [0.0;0.0; 0.0];
w_i = [0;0];

i = 0; tau = 0;
tic;

MTrackerDriver('setOdometry', q_i); 
wait(tau, 0.1);
data = MTrackerDriver('read');

done = zeros(10);


tic;
disp('Robot is started.');

% ----------------------FORMAT KOMENDY----------------------
% Wspolrzedne x LUB parametr
% Wspolrzedne y LUB parametr
% Kat th LUB parametr
% ------------
% Czy nowy rozkaz? 0 - odczyt danych, 1 - nowy rozkaz
% Czy czyscic bufor kolejki? 0 - nie, 1 - czyscic czyli przerwac aktualne zadanie
% Tryb sterowania 0-3: 0 - pozycja, 1 - orientacja, 2 - pozycja z ustalona predkoscia, 3 - tryb spowolnienia do 0 (slow)
% Wybor parametru 0-7: 0 - brak zmiany ustawien, 1-7 - ustawianie parametrow wg. tablicy parametrow
% ----------------------------------------------------------

% ----------------------TABLICA PARAMETROW------------------
% Parametr | x                  | y                  | th
% 0        |            brak zmiany parametru
% 1        | wl_max_orientation | wr_max_orientation | error_orientation
% 2        | k                  | d                  | error_position
% 3        | V_const            | eps                | error_position
% 4        | SlowCoef           | SlowThreshold      | error_position
% 5        | wl_max             | wr_max             | error_position
% 6        |               nieuzywane
% 7        |               nieuzywane
% ----------------------------------------------------------

while (tau < Tf)
    
    i = i+1;
    tau = toc;

    
   if (done(1) == 0)
   MTrackerDriver('highLevelControl',[0.2; 0.1; 0.1; 0;1;0; 2]); %x, y, th, zadanie punktu (jesli 0 to wysylamy pusta ramke)
   done(1) = 1;
   % elseif (done(2) == 0)
   % MTrackerDriver('highLevelControl',[0.2; 0.2; 0.01; 0;1;0; 2]);
   % done(2) = 1;
   else
    MTrackerDriver('highLevelControl',[0.5; 0.5; 0.01; 0;0;0; 0]);
   end
    
    if (done(4) == 0  )
         MTrackerDriver('highLevelControl',[0.8; 0.9; 3.14/2; 1;0;2; 0]);
         done(4) = 1;
    end
    
    
    %{
    if (done(5) == 0  && tau > 3 )
         MTrackerDriver('highLevelControl',[-0.3; -0.2; -0.4; 1;1;0; 0]);
         done(5) = 1;
    end
    if (done(6) == 0  && done(5) )
         MTrackerDriver('highLevelControl',[-0.3; -0.2; -0.4; 1;0;1; 0]);
         done(6) = 1;
    end
    if (done(3) == 0  )
         MTrackerDriver('highLevelControl',[0.1; -1.2; -0.4; 1;0;2; 0]);
         done(3) = 1;
    end
    if (done(7) == 0 && done(3) )
         MTrackerDriver('highLevelControl',[-1.1; -1.2; -0.4; 1;0;2; 0]);
         done(7) = 1;
    end
    
    %}
    %{
    if (done(5) == 0)
         MTrackerDriver('highLevelControl',[0.05; 0.5; 0.01; 0;1;1; 5]);
         done(5) = 1;
    end
     
%}
    
    %{
    
    
    if (done(1) == 0 && tau > 5 )
         MTrackerDriver('highLevelControl',[0.8; -1.1; -0.4; 1;1;1; 0]);
         done(1) = 1;
    end
    if (done(3) == 0 && done(1) == 1 )
         MTrackerDriver('highLevelControl',[-0.2; -0.5; -0.4; 1;0;0; 0]);
         done(3) = 1;
    end
    if (done(4) == 0 && done(3) == 1 )
         MTrackerDriver('highLevelControl',[0.4; 0.3; -0.4; 1;0;2; 0]);
         done(4) = 1;
    end
    %}
     
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