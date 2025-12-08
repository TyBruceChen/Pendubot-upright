function u_traj=OL_TO()
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


display(length(t_traj));
display(t_traj(end))
end