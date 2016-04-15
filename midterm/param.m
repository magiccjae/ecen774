clear;
% inverted Pendulum parameter file

% actual system parameters used in simulation model
% randomly select a
P.a = 4*rand-2;

% initial conditions
P.y0 = 0;
P.ydot0 = 0;
P.z0 = 0;

% sample rate for controller
P.Ts = 0.01;
% dirty derivative gain
P.tau = 0.5;

