clear; clc; close all;

%% ===========================
%      Pendubot: Part A
%   Trajectory Optimization
% ============================

% ---- bounds ----
q1_min = -2*pi; q1_max =  2*pi;
q2_min = -2*pi; q2_max =  2*pi;
dq_min = -10;   dq_max =  10;

u_min  = -12;    u_max  =  12;   % 

x0 = [-pi/2; 0; 0; 0];        % both links down
xF = [pi/2; 0; 0; 0];       % first up, second down 

problem.func.dynamics = @(t,x,u) pendubot_dynamics(t,x,u);
problem.func.pathObj  = @(t,x,u) u.^2;     % ∫ u^2 dt

% time bounds
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low   = 3;
problem.bounds.finalTime.upp   = 3;

% state bounds
problem.bounds.initialState.low = x0;
problem.bounds.initialState.upp = x0;
problem.bounds.finalState.low   = xF;
problem.bounds.finalState.upp   = xF;

problem.bounds.state.low = [q1_min; q2_min; dq_min; dq_min];
problem.bounds.state.upp = [q1_max; q2_max; dq_max; dq_max];

% control bounds
problem.bounds.control.low = u_min;
problem.bounds.control.upp = u_max;

% initial guess
problem.guess.time    = [0 10];
problem.guess.state   = [x0 xF];
problem.guess.control = [0 0];

% options
problem.options.method = 'trapezoid';
problem.options.trapezoid.nGrid = 100;

problem.options.nlpOpt = optimset(...
    'Display','iter',...
    'MaxFunEvals',5e5,...
    'MaxIter',1e4);

% ---- solve TO ----
fprintf('Solving Trajectory Optimization...\n');
soln = optimTraj(problem);
t_traj = soln.grid.time;
x_traj = soln.grid.state;
u_traj = soln.grid.control;

animatePendubot(t_traj, x_traj);

%% ===========================
% 2. LQR DESIGN (Stabilization)
% ============================
fprintf('Calculating LQR Gain...\n');

% We need the A and B matrices at the top (xF).
% Since calculating partial derivatives manually is messy, 
% we use numerical linearization (finite differences).

delta = 1e-2;
A = zeros(4,4);
B = zeros(4,1);
v_eq = 0; % Equilibrium torque at top is 0 for this model
x_eq = xF;

% Compute A matrix (df/dx)
f0 = pfl_virtual_dynamics(x_eq, v_eq);
for i = 1:4
    x_pert = x_eq;
    x_pert(i) = x_pert(i) + delta;
    f_pert = pfl_virtual_dynamics(x_pert, v_eq);
    A(:,i) = (f_pert - f0) / delta;
end

% Compute B matrix (df/du)
v_pert = v_eq + delta;
f_pert = pfl_virtual_dynamics(x_eq, v_pert);
B = (f_pert - f0) / delta;

% LQR Weights
Q = diag([100, 100, 10, 10]); % Penalize position errors heavily
R = 1;                        % Cheap control authority
K = lqr(A, B, Q, R);

disp('LQR Gain K:'); disp(K);

%% ===========================
% 3. SIMULATION (Swing-up + LQR)
% ============================
fprintf('Simulating with LQR Switch...\n');

% Interpolator for the Open-Loop part
u_interp_func = @(t) interp1(t_traj, u_traj, t, 'pchip', 'extrap');

% Simulation Wrapper
% We pass the Target state (xF), the Trajectory, and the LQR Gain K
ctrl_fun = @(t,x) hybrid_controller(t, x, xF, u_interp_func, K, u_max);


ode_fun  = @(t,x) pendubot_dynamics(t, x, ctrl_fun(t,x));
% Run Simulation (Extend time to see balancing)
t_end_sim = t_traj(end) + 1; 
disp('Start ode45');
[t_sim, x_sim] = ode45(ode_fun, [0:0.001: t_end_sim], x0);
disp('Over ode45');

% Reconstruct Control Input for Plotting
u_sim_log = zeros(length(t_sim), 1);
mode_log  = zeros(length(t_sim), 1); % 0=Swing, 1=LQR
for i = 1:length(t_sim)
    [u_val, mode] = hybrid_controller(t_sim(i), x_sim(i,:)', xF, u_interp_func, K, u_max);
    u_sim_log(i) = u_val;
    mode_log(i)  = mode;
end

%% ===========================
% 4. PLOTTING
% ============================
figure('Color','w','Position',[100 100 800 600]);

subplot(5,1,1);
plot(t_traj, x_traj(1,:), 'r--', 'LineWidth', 1.5); hold on;
plot(t_sim, x_sim(:,1), 'b', 'LineWidth', 2);
yline(xF(1), 'k:');
ylabel('q_1 (rad)'); title('Shoulder Angle'); legend('Planned','Simulated'); grid on;

subplot(5,1,2);
plot(t_traj, x_traj(2,:), 'r--', 'LineWidth', 1.5); hold on;
plot(t_sim, x_sim(:,2), 'b', 'LineWidth', 2);
yline(xF(2), 'k:');
ylabel('q_2 (rad)'); title('Elbow Angle'); grid on;

subplot(5,1,3);
plot(t_traj, x_traj(3,:), 'r--', 'LineWidth', 1.5); hold on;
plot(t_sim, x_sim(:,3), 'b', 'LineWidth', 2);
yline(xF(2), 'k:');
ylabel('dq_1 (rad/s)'); title('Shoulder Speed'); grid on;

subplot(5,1,4);
plot(t_traj, x_traj(4,:), 'r--', 'LineWidth', 1.5); hold on;
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
yline(u_max,'r--'); yline(u_min,'r--');
ylabel('Torque (Nm)'); xlabel('Time (s)'); title('Control Input'); grid on;

animatePendubot(t_sim, x_sim');

%%
function [u, mode] = hybrid_controller(t, x, xF, u_traj_func, K, u_max)
    persistent lqr_engaged
    
    % Reset the memory if it's the start of simulation (t=0)
    % or if the variable hasn't been created yet.
    if isempty(lqr_engaged) || t == 0
        lqr_engaged = false;
    end

    % 1. Calculate Error
    error = x - xF;
    
    % 2. Define Switching Condition
    % Switch to LQR if position is close AND velocity is manageable
    e_pos = atan2(sin(error(1:2)), cos(error(1:2)));
    pos_err = norm(e_pos);
    e_vel = error(3:4);
    vel_err = norm(e_vel);

    %disp(error)
    % Thresholds: e.g., within 0.3 rad (approx 17 deg) and moving slowly
    if ((pos_err >= 0.6) || (vel_err >= 10.0)) && (lqr_engaged == false)
        % --- SWING UP MODE (Trajectory Playback) ---
        mode = 0;
        u = u_traj_func(t); 
        % disp('TO Activated');
    else
        lqr_engaged = true;
        % --- LQR MODE ---
        mode = 1;
        v = -K*[e_pos; e_vel];
        u = inverse_dynamics_pfl(x,v);
        %disp('PFL-LQR Activated');
    end
    
    % 3. Saturation (Physical Limits)
    u = max(min(u, u_max), -u_max);
end

% --- PFL Helper: Calculates dynamics assuming q1_ddot = v ---
function dx = pfl_virtual_dynamics(x, v)
    % Unpack
    q1 = x(1); q2 = x(2); dq1 = x(3); dq2 = x(4);
    
    % Get Matrices
    [D, H, Phi] = get_matrices(x);
    d21 = D(2,1); d22 = D(2,2);
    h2 = H(2); phi2 = Phi(2);
    
    % If we force ddq1 = v, what happens to ddq2?
    % Equation 2: d21*v + d22*ddq2 + h2 + phi2 = 0
    % ddq2 = -inv(d22) * (d21*v + h2 + phi2)
    
    ddq2 = -(d21*v + h2 + phi2) / d22;
    
    dx = [dq1; dq2; v; ddq2];
end

% --- PFL Helper: Calculates Torque to achieve q1_ddot = v ---
function tau = inverse_dynamics_pfl(x, v)
    % Unpack
    % q1 = x(1); q2 = x(2); dq1 = x(3); dq2 = x(4);
    
    [D, H, Phi] = get_matrices(x);
    d11 = D(1,1); d12 = D(1,2);
    d21 = D(2,1); d22 = D(2,2);
    h1 = H(1); h2 = H(2);
    phi1 = Phi(1); phi2 = Phi(2);
    
    % From PFL derivation (Collocated Linearization):
    % tau = M_bar * v + N_bar
    
    % M_bar = d11 - d12*inv(d22)*d21
    m_bar = d11 - (d12 * d21 / d22);
    
    % N_bar = (h1 + phi1) - d12*inv(d22)*(h2 + phi2)
    n_bar = (h1 + phi1) - (d12 / d22) * (h2 + phi2);
    
    tau = m_bar * v + n_bar;
end

% --- Standard Matrices Helper ---
function [D, H, Phi] = get_matrices(x)
    q1 = x(1); q2 = x(2); dq1 = x(3); dq2 = x(4);
    
    g   = 9.81;
    Th1 = 3.08e-2; Th2 = 1.06e-2; Th3 = 9.5e-3;
    Th4 = 2.087e-1; Th5 = 6.3e-2;
    
    c2 = cos(q2); s2 = sin(q2);
    c1 = cos(q1); c12 = cos(q1+q2);

    D = [Th1 + Th2 + 2*Th3*c2,  Th2 + Th3*c2;
         Th2 + Th3*c2,          Th2];
     
    h_term = Th3 * s2;
    H = [-h_term * dq2 * (2*dq1 + dq2);
          h_term * dq1^2];
      
    Phi = [Th4*g*c1 + Th5*g*c12;
           Th5*g*c12];
end


