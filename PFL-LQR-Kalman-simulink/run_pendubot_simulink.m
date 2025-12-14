%% run_pendubot_sim
%
% Sets initial conditions and needed variables, runs a simulink
% (offline) simulation of the pendubot, and animates the resulting
% motion.
%
% This code can be modified to implement a Kalman filter (or other
% state estimation) in order to estimate the full state of the 
% pendubot (i.e., both joint angles AND joint velocities) based on
% encoder readings.
%
% The dynamics use forward Euler integration of the nonlinear
% equations of motion (EOM) from the Pendubot Manual document, which
% can be found on the "Final Project / Pendubot" page on canvas.
%
% Katie Byl, UCSB, ECE 238
clear; clc;
T = 1e-3; % Sample time (for simulating A/D and D/A discrete-time effects)

X0 = [-90*pi/180; 0*pi/180; 0; 0]; % q1; q2; dq1; dq2 (rad, and rad/s)
xF = [pi/2; 0; 0; 0];

noise_avg = 0.00; noise_std = 0.001;
Tsim = 4; % How long the simulation will run, in simulink
%%
% Ad = [1.000000000006551                   0   0.000999999993923                   0
%     0   1.000000000000000                   0   0.001000000000000
%     -0.067013475002497   0.024854104048693   1.000000000000000                   0
%     0.068768004486457  -0.105433725600063                   0   1.000000000000000];
% 
% Bd =[                   0
%     0
%     0.044871523515218
%     -0.085086568175084];

[K, Ad, Bd]=k_LQR(xF);
kalman = kalman_const(Ad, Bd);

%%
data = sim('Pendubot_PFL_LQR.slx');

tout = data.simout.Time;
xout = data.simout.Data;
xout = reshape(xout,5,length(tout))';
tau_output = xout(:,5);    % torque (tau)
xout = xout(:,1:4); % 4 states

figure(1); clf
animatePendubot(tout,xout) % animate the motion

% plot the angles, angular velocities, and torque (constant)

figure(2); clf
subplot(3,1,1); plot(tout,xout(:,1:2)); legend('q_1','q_2');
set(gca,'FontSize',18); grid on
subplot(3,1,2); plot(tout,xout(:,3:4)); legend('dq_1/dt','dq_2/dt');
set(gca,'FontSize',18); grid on
subplot(3,1,3); plot(tout,tau_output); legend('tau');
xlabel('Time (s)')
set(gca,'FontSize',18); grid on



