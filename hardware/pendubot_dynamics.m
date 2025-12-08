function dx = pendubot_dynamics(t, x, u)
% Vectorized Pendubot dynamics for OptimTraj
% state x = [q1; q2; dq1; dq2], control u = shoulder torque (scalar)

% ----parameters----
g  = 9.8;

Th1 = 3.08e-2;
Th2 = 1.06e-2;
Th3 = 9.5e-3;
Th4 = 2.087e-1;
Th5 = 6.3e-2;

N = size(x,2);
dx = zeros(4,N);
T= 0.1;

for k = 1:N
    q1  = x(1,k);  q2  = x(2,k);
    dq1 = x(3,k);  dq2 = x(4,k);
    tau = u(1,k);          % scalar

    cosq1 = cos(q1);
    cosq2 = cos(q2); 
    sinq2 = sin(q2);
    cosq1q2 = cos(q1+q2);

    d11 = Th1 + Th2 + 2 * Th3 * cosq2;
    d12 = Th2 + Th3 * cosq2;
    d22 = Th2;
    D = [d11 d12; d12 d22];

    h1 = -Th3 * sinq2 * dq2 * (dq1+dq2);
    h2 = Th3 * sinq2 * dq1^2;
    H = [h1; h2];

    phi1 = Th4 * g * cosq1 + Th5 * g * cosq1q2;
    phi2 = Th5 * g * cosq1q2;
    Phi = [phi1; phi2];

    Tau = [tau; 0]; % torque (N*m) applied to first link

    ddq = D \ (Tau - H - Phi);

    dX = [dq1; dq2; ddq];

    dx(:, k) = dX;
end
end
