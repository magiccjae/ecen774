clear; close all; clc;
% initial conditions
P.phi0 = 10*pi/180;
P.p0 = 0;

P.omega_n = 1;
P.zeta = 0.7;

% sample rate for controller
P.Ts = 0.01;

P.B = [0; 1];
P.Q = [1 0; 0 10];
value = 10;
value2 = 10;
P.gamma_x = [value 0; 0 value];
P.gamma_r = value;
P.gamma_theta = [value2 0 0; 0 value2 0; 0 0 value2];
% P.gamma_x = [100 0; 0 100];
% P.gamma_r = 100;
% P.gamma_theta = [100 0 0; 0 100 0; 0 0 100];

P.a_ref = [0 1; -P.omega_n^2 -2*P.zeta*P.omega_n];
P.b_ref = [0; P.omega_n^2];

P.P = lyap(P.a_ref', P.Q);