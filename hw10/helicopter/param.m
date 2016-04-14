clear; close all; clc;
% initial conditions
P.q0 = 0;

% sample rate for controller
P.Ts = 0.01;

P.gamma_q = 100;
P.gamma_q_cmd = 100;
P.gamma_theta = 8;