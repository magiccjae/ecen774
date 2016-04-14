clear; close all; clc;
% initial conditions
P.phi0 = 10*pi/180;
P.p0 = 0;

% sample rate for controller
P.Ts = 0.01;

% baseline controller
P.A = [0 1 0; 0 0 1; 0 -0.018 0.015];
P.B = [0; 0; 1];
P.B_ref = [-1; 0; 0];
lqr_Q = eye(3);
lqr_R = eye(1);

[K,S,E] = lqr(P.A, P.B, lqr_Q, lqr_R);
P.K = K;

% adaptive controller
value = 100;
P.gamma_theta = [value 0 0; 0 value 0; 0 0 value];

P.a_ref = P.A - P.B*P.K;

P.Q_ref = diag([100; 100; 100]);
P.P_ref = lyap(P.a_ref', P.Q_ref);
