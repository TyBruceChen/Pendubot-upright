function [Kdlqr, Ad, Bd] = k_LQR(xF)
    Ts = 1e-3;

    % Linearize continuous-time model at upright
    [Ac, Bc] = linearize_pendubot_upright(xF);

    % Discretize to match Kalman script form
    [Ad, Bd] = c2d_zoh(Ac, Bc, Ts);

    % Weights (you can reuse yours as a starting point)
    Q = diag([500, 500, 50, 50]);
    R = 1;

    % Discrete LQR (note: dlqr, not lqr)
    Kdlqr = dlqr(Ad, Bd, Q, R);
end

function [Ac, Bc] = linearize_pendubot_upright(xF)
    taueq = 0;
    delta = 1e-6;

    % f0
    f0 = pendubot_dynamics(0, xF, taueq);

    % A = df/dx
    Ac = zeros(4,4);
    for i = 1:4
        xpert = xF;
        xpert(i) = xpert(i) + delta;
        fpert = pendubot_dynamics(0, xpert, taueq);
        Ac(:,i) = (fpert - f0)/delta;
    end

    % B = df/dtau
    fpert_u = pendubot_dynamics(0, xF, taueq + delta);
    Bc = (fpert_u - f0)/delta;
end

function [Ad, Bd] = c2d_zoh(Ac, Bc, Ts)
    n = size(Ac,1);
    M = [Ac, Bc; zeros(1,n), 0];
    Md = expm(M*Ts);
    Ad = Md(1:n, 1:n);
    Bd = Md(1:n, n+1);
end