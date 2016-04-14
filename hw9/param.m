% Differential flatness example for ECE 774
%
% Modified: 4/15/2014 - R. Beard
%
%
clear; close all; clc;

P.m = 1;
P.Jz = 0.01;

% initial position and velocity
P.rx_0 = rand*80 + 10;
P.ry_0 = rand*80 + 10;
P.vx_0 = rand*5;
P.vy_0 = rand*5;

% final position and velocity
P.rx_T = rand*80 + 10;
P.ry_T = rand*80 + 10;
P.vx_T = rand*5;
P.vy_T = rand*5;

P.acc_max = 1;

P.T = compute_T(P);
P.C = compute_coefficient(P.T, P);

y_traj = zeros(2,501);
time = 0:P.T/500:P.T;

for i = 1:501
   t = time(i);
   y_traj(:,i) =  [1 t t^2 t^3 0 0 0 0; 0 0 0 0 1 t t^2 t^3] * P.C;
end
figure(2);
plot(y_traj(1,:), y_traj(2,:));
axis([0 100 0 100]);

