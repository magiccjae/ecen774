clear; close all; clc;

% actual system parameters
    AP.m     = 10;   % kg
    AP.mu1   = 25;   % kg/m
    AP.mu2   = 15;   % kg s^2 / m^3
    AP.alpha = 0.8;  % unitless
    AP.vc    = 0.5;  % m/s
    P.Tmax   = 50;     % N
    

% initial conditions
    AP.p0 = 0; % m
    AP.v0 = 0; % m/s

% parameters known to the controller
    % size of uncertainty in parameters
    S = 0.0;  % up to 100*S % change in parameters
    P.m     = AP.m*(1+2*S*rand-S);      % kg
    P.mu1   = AP.mu1*(1+2*S*rand-S);    % kg/m
    P.mu2   = AP.mu2*(1+2*S*rand-S);    % kg s^2 / m^3
    P.alpha = AP.alpha*(1+2*S*rand-S);  % unitless
    P.vc    = AP.vc*(1+2*S*rand-S);     % m/s

% sample rate for controller
    P.Ts = 0.01;
% parameter for dirty derivative
    P.tau = 0.05;

% select which controller to use in simulation
    P.control_selection_flag = 5;  % 1==PD, 2==backstepping, 3==feedback linearization, 4==sliding mode, 5==adaptive
    P.num_adaptive_param = 1;  % one except for option 5
    
% parameters for each controller
    switch P.control_selection_flag,
        case 1, % PD control
            P.kp = 10;
            P.kd = 10;
            AP.vc = 0;      % vc=0 in the problem
        case 2, % backstepping
            P.k1 = 10;      % tuning parameter
            P.k2 = 10;      % tuning parameter
%             P.m     = AP.m;      % kg
%             P.mu1   = AP.mu1;    % kg/m
%             P.mu2   = AP.mu2;    % kg s^2 / m^3
%             P.alpha = AP.alpha;  % unitless
%             P.vc    = AP.vc;     % m/s
            
        case 3, % feedback linearization
            P.k1 = 10;
            P.k2 = 10;
        case 4, % sliding mode
            P.ke = 5;
            P.beta = 1;
        case 5, % adaptive
            % baseline controller
            P.A = [0 1 0; 0 2*P.mu1*P.vc/P.m+4*P.mu2*P.vc^3/P.m 0; 1 0 0];
            P.B = [0; P.alpha/P.m; 0];
            P.B_ref = [0; 0; -1];
            lqr_Q = 1000000*eye(3);
            lqr_R = eye(1);
            
            [K,S,E] = lqr(P.A, P.B, lqr_Q, lqr_R);
            P.K = K;
            
            % adaptive gains
%             P.Gam = diag([]);
%             P.num_adaptive_param = size(P.Gam,1);

    end


        
    


