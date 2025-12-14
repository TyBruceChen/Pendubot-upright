function Lpk_ss = kalman_const(Ad, Bd)
    
    T = 1e-3;  % sample time
    
    % Ad = [1.000000000006551                   0   0.000999999993923                   0
    %     0   1.000000000000000                   0   0.001000000000000
    %     -0.067013475002497   0.024854104048693   1.000000000000000                   0
    %     0.068768004486457  -0.105433725600063                   0   1.000000000000000];
    % 
    % Bd =[                   0
    %     0
    %     0.044871523515218
    %     -0.085086568175084];
    
    C = [1 0 0 0; 0 1 0 0]; % only the angles are measured
    
    Xeq = [1.570796326794897
        0
        0
        0]; % As a reality check, Ad and Bd are for THIS equilibrium, i.e.
    % with both link DOWN
    
    %% You may wish to reason more carefully about the noise covariances
    %  (for example, if the noise WERE due to some u pulse, there would
    %  be some off-diagonal terms in Qss...)
    Rss = diag([2*pi/5000, 2*pi/10000].^2); % estimate SENSOR NOISE COV
    Qss = diag([Bd*.01].^2); % estimate PROCESS NOISE COVARIANCE
    Qss(1,1) = 1e-9; Qss(2,2) = 1e-9; % optional, add a tiny bit
    %Qss = diag([1 1 1 1]*1e-7);
    P0 = diag([1 1 1 1]);
    Pplus = P0;
    for n=1:1000
        Pminus = Ad*Pplus*Ad' + Qss;
        Lpk = (Pminus*C') * (C * Pminus * C' + Rss)^(-1);
        Pplus = (eye(4) - Lpk*C) * Pminus * (eye(4) - Lpk*C)' + Lpk*Rss*Lpk';
    end
    
    Lpk_ss = Lpk; % steady-state Kalman filter gains
    Kalman_poles = eig(Ad - Lpk*C);  % z-plane estimator poles
    svalues_Kalman_poles = log(Kalman_poles)./T; % mapped to s plane

end
