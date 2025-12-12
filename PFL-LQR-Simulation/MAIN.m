clear;clc;
xF = [pi/2; 0; 0; 0];
uF = 0;
t_end_sim = 2.5;
u_max = 12;
u_min = -12;

x0 = [-pi/2; 0; 1; 1] + 1e-4;
global lqr_engaged;
lqr_engaged = false;

%% ===========================
% 2. LQR DESIGN (Stabilization)
% ============================
fprintf('Calculating LQR Gain...\n');

% We need the A and B matrices at the top (xF).
% Since calculating partial derivatives manually is messy, 
% we use numerical linearization (finite differences).

delta = 1e-1;
A = zeros(4,4);
B = zeros(4,1);
v_eq = 1e-4; % Equilibrium torque at top is 0 for this model
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
%%
ode_controller = @(x) hybrid_controller(x, xF, K, u_max);
ode_fun = @(t, x) pendubot_dynamics(t,x, ode_controller(x));

[t_sim, x_sim] = ode45(ode_fun, [0:0.001: t_end_sim], x0);

% Reconstruct Control Input for Plotting
u_sim_log = zeros(length(t_sim), 1);
mode_log  = zeros(length(t_sim), 1); % 0=Swing, 1=LQR
lqr_engaged = false;
for i = 1:length(t_sim)
    [u_val, mode] = hybrid_controller(x_sim(i,:)', xF, K, u_max);
    disp(mode)
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
    uF=0; kp = 250;kd = 30;
    global lqr_engaged;
    % 1. Calculate Error
    error = x - xF;
    
    % 2. Define Switching Condition
    e_pos = atan2(sin(error(1:2)), cos(error(1:2)));
    pos_err = norm(e_pos);
    e_vel = error(3:4);
    vel_err = norm(e_vel);

    %disp(error)
    if ((pos_err >= 0.6) || (vel_err >= 15.0)) && (lqr_engaged == false)
        % --- SWING UP MODE (Trajectory Playback) ---
        mode = 0;
        u = pendubot_pfl_controller(x, xF(1), xF(3), uF, kp, kd);
        %disp('PFL Activated');
        % disp(mode);
    else
        lqr_engaged = true;
        % --- LQR MODE ---
        mode = 1;
        v = -K*[e_pos; e_vel];
        u = inverse_dynamics_pfl(x,v);
        % disp('LQR Activated');
    end
    
    % 3. Saturation (Physical Limits)
    u = max(min(u, u_max), -u_max);
    disp(mode);
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
