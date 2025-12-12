function tau = pendubot_pfl_controller(x, q1d, dq1d, ddq1d, kp, kd)

% x = [q1;q2;dq1;dq2]
q1  = x(1);  q2  = x(2);
dq1 = x(3);  dq2 = x(4);

% Parameters (must match pendubot_dynamics.m)
g  = 9.8;
Th1 = 3.08e-2;
Th2 = 1.06e-2;
Th3 = 9.5e-3;
Th4 = 2.087e-1;
Th5 = 6.3e-2;

cosq1 = cos(q1);
cosq2 = cos(q2);
sinq2 = sin(q2);
cosq1q2 = cos(q1+q2);

% Inertia terms (same as dynamics file)
d11 = Th1 + Th2 + 2*Th3*cosq2;
d12 = Th2 + Th3*cosq2;
d22 = Th2;

% Coriolis/centrifugal terms
h1 = -Th3*sinq2*dq2*(dq1+dq2);
h2 =  Th3*sinq2*dq1^2;

% Gravity terms
phi1 = Th4*g*cosq1 + Th5*g*cosq1q2;
phi2 = Th5*g*cosq1q2;

% PFL scalars
Delta = d11 - (d12^2)/d22;
beta  = (h1 + phi1) - (d12/d22)*(h2 + phi2);

% Virtual input v to impose ddq1 = v
v = ddq1d - kd*(dq1 - dq1d) - kp*(q1 - q1d);

% Shoulder torque
tau = Delta*v + beta;

end