clear; close all; clc;

P.alpha = 1;
P.lambda = 0.01;    % focal length. 10mm
P.z = rand*100 + 50;     % random number between 50 to 150
P.theta = 1/P.z;
P.gamma_theta = 10;

% sample rate for controller
P.Ts = 0.01;

P.v_cx = rand*20;    
P.v_cy = rand*20;
P.v_cz = 0;

% pixel_max = 800;
% P.u_max = pixel_max/2;
% P.u_min = -pixel_max/2;
% P.w_max = pixel_max/2;
% P.w_min = -pixel_max/2;

image_max = 0.3;
P.u_max = image_max/2;
P.u_min = -image_max/2;
P.w_max = image_max/2;
P.w_min = -image_max/2;

% initial conditions
% P.u_ref0 = (2*rand-1)*image_max/2;
% P.w_ref0 = (2*rand-1)*image_max/2;
% 
% P.u_actual0 = (2*rand-1)*image_max/2;
% P.w_actual0 = (2*rand-1)*image_max/2;

P.u_ref0 = 0.15;
P.w_ref0 = -0.15;

P.u_actual0 = -0.15;
P.w_actual0 = 0.15;
