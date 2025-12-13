clear;clc;
xF = [pi/2; 0; 0; 0];
uF = 0;
t_end_sim = 3;
u_max = 12;
u_min = -12;

x0 = [-pi/2; 0; 0; 0];
global lqr_engaged;
lqr_engaged = false;

%% ===========================
% 2. LQR DESIGN (Stabilization)
% ============================
fprintf('Calculating LQR Gain...\n');

[Kdlqr, Ad, Bd, xeq] = lqr_upright_discrete();

disp('Discrete LQR Gain K:'); disp(Kdlqr);
%%
ode_controller = @(x) hybrid_controller(x, xF, Kdlqr, u_max);
ode_fun = @(t, x) pendubot_dynamics(t,x, ode_controller(x));

[t_sim, x_sim] = ode45(ode_fun, [0:0.001: t_end_sim], x0);

% Reconstruct Control Input for Plotting
u_sim_log = zeros(length(t_sim), 1);
mode_log  = zeros(length(t_sim), 1); % 0=Swing, 1=LQR
lqr_engaged = false;
for i = 1:length(t_sim)
    [u_val, mode] = hybrid_controller(x_sim(i,:)', xF, Kdlqr, u_max);
    % disp(mode)
    u_sim_log(i) = u_val;
    mode_log(i)  = mode;
end

%% ===========================
% 4. PLOTTING
% ============================
figure('Color','w','Position',[100 100 800 600]);

subplot(5,1,1);
plot(t_sim, x_sim(:,1), 'b', 'LineWidth', 2);
yline(xF(1), 'k:');
ylabel('q_1 (rad)'); title('Shoulder Angle'); grid on;

subplot(5,1,2);
plot(t_sim, x_sim(:,2), 'b', 'LineWidth', 2);
yline(xF(2), 'k:');
ylabel('q_2 (rad)'); title('Elbow Angle'); grid on;

subplot(5,1,3);
plot(t_sim, x_sim(:,3), 'b', 'LineWidth', 2);
yline(xF(2), 'k:');
ylabel('dq_1 (rad/s)'); title('Shoulder Speed'); grid on;

subplot(5,1,4);
plot(t_sim, x_sim(:,4), 'b', 'LineWidth', 2);
yline(xF(2), 'k:');
ylabel('dq_2 (rad/s)'); title('Elbow Speed'); grid on;

subplot(5,1,5);
plot(t_sim, u_sim_log, 'k', 'LineWidth', 1.5); hold on;
% Shade the LQR region
area_indices = find(mode_log == 1);
if ~isempty(area_indices)
    x_area = [t_sim(area_indices(1)) t_sim(area_indices(end)) t_sim(area_indices(end)) t_sim(area_indices(1))];
    y_area = [u_min u_min u_max u_max];
    fill(x_area, y_area, 'g', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
    text(t_sim(area_indices(1))+0.1, u_max-2, 'LQR Active', 'Color', 'g', 'FontWeight', 'bold');
end
ylabel('Torque (Nm)'); xlabel('Time (s)'); title('Control Input'); grid on;

animatePendubot(t_sim, x_sim);
%%

function [u, mode] = hybrid_controller(x, xF, K, u_max)
    uF=0; kp = 3650;kd = 180; %best 3780, 180
    global lqr_engaged;
    % 1. Calculate Error
    error = x - xF;
    
    % 2. Define Switching Condition
    e_pos = atan2(sin(error(1:2)), cos(error(1:2)));
    pos_err = norm(e_pos);
    e_vel = error(3:4);
    vel_err = norm(e_vel);

    %disp(error)
    if ((pos_err >= 0.8) || (vel_err >= 20.0)) && (lqr_engaged == false)
        % --- SWING UP MODE (Trajectory Playback) ---
        mode = 0;
        u = pendubot_pfl_controller(x, xF(1), xF(3), uF, kp, kd);
        %disp('PFL Activated');
        % disp(mode);
    else
        lqr_engaged = true;
        % --- LQR MODE ---
        mode = 1;
        u = -K * [e_pos; e_vel];
        % disp('LQR Activated');
    end
    
    % 3. Saturation (Physical Limits)
    u = max(min(u, u_max), -u_max);
end

function [Ac, Bc] = linearize_pendubot_upright()
    % Upright equilibrium
    xeq  = [pi/2; 0; 0; 0];
    taueq = 0;

    delta = 1e-6;

    % f0
    f0 = pendubot_dynamics(0, xeq, taueq);

    % A = df/dx
    Ac = zeros(4,4);
    for i = 1:4
        xpert = xeq;
        xpert(i) = xpert(i) + delta;
        fpert = pendubot_dynamics(0, xpert, taueq);
        Ac(:,i) = (fpert - f0)/delta;
    end

    % B = df/dtau
    fpert_u = pendubot_dynamics(0, xeq, taueq + delta);
    Bc = (fpert_u - f0)/delta;
end

function [Ad, Bd] = c2d_zoh(Ac, Bc, Ts)
    n = size(Ac,1);
    M = [Ac, Bc; zeros(1,n), 0];
    Md = expm(M*Ts);
    Ad = Md(1:n, 1:n);
    Bd = Md(1:n, n+1);
end

function [Kdlqr, Ad, Bd, xeq] = lqr_upright_discrete()
    Ts = 1e-3;

    % Linearize continuous-time model at upright
    [Ac, Bc] = linearize_pendubot_upright();

    % Discretize to match Kalman script form
    [Ad, Bd] = c2d_zoh(Ac, Bc, Ts);

    % Upright equilibrium
    xeq = [pi/2; 0; 0; 0];

    % Weights (you can reuse yours as a starting point)
    Q = diag([500, 500, 50, 50]);
    R = 1;

    % Discrete LQR (note: dlqr, not lqr)
    Kdlqr = dlqr(Ad, Bd, Q, R);
end