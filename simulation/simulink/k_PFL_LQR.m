function K=k_PFL_LQR()

xF = [pi/2; 0; 0; 0]; 
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
