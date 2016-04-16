clear; close all; clc;

% actual system parameters
    AP.m1 = 0.35;  % kg
    AP.m2 = 2;     % kg
    AP.L = 0.5;    % m
    AP.g = 9.8;    % m/s^2

% initial conditions
    AP.z0        = 0;
    AP.zdot0     = 0;
    AP.theta0    = 0;
    AP.thetadot0 = 0;

% parameters known to the controller
    % size of uncertainty in parameters
    alpha = 1.0;  % up to 100*alpha % change in parameters
    P.m1 = AP.m1*(1+2*alpha*rand-alpha);  % kg
    P.m2 = AP.m2*(1+2*alpha*rand-alpha);  % kg
    P.L  = AP.L*(1+2*alpha*rand-alpha);   % m
    P.g  = AP.g; % m/s^2

% sample rate for controller
    P.Ts = 0.01;
% parameter for dirty derivative
    P.tau = 0.05;
% limits on force
    P.Fmax = 15; % N

% state space model based on Jacobian linearization
    % states are x=[theta, z, thetadot, zdot]
    Ap = [...
        0, 0, 1, 0;...
        0, 0, 0, 1;...
        0, -P.m1*P.g/((P.m2*P.L^2)/3+P.m1*(P.L/2)^2), 0, 0;...
        -P.g, 0, 0, 0;...
        ];
    Bp = [0; 0; P.L/(P.m2*P.L^2/3+P.m1*P.L^2/4); 0; ];
    Cp = [...
        1, 0, 0, 0;...
        0, 1, 0, 0;...
        ];

% form augmented system to add integrator
    Cout = [0,1,0,0];
    P.A = [Ap, zeros(4,1); Cout, 0];
    P.B = [Bp; 0];
    
% design LQR controller
    Q = diag([...
            0.01/(5*pi/180);...  % penalty on theta
            0.01;...  % penalty on z
            5;...  % penalty on thetadot
            5;...  % penalty on zdot
            10;...  % penalty on integrator
            ]);
    R = 1/P.Fmax;  % penalty on F
    [P.K, P.Pref] = lqr(P.A,P.B,Q,R); 

% construct reference model
    P.Aref = P.A-P.B*P.K;
    P.Bref = [0; 0; 0; 0; -1];
    P.Cref = [Cout, 0];
    
% adaptive gains
    P.Gam = 10*diag([...
        1;...  %baseline controller
        1;...  % constant
        1;...  % theta
        1;...  % z-zref
        1;...  % thetadot
        1;...  % zdot
        1;...  % z*zdot*thetadot/(P.m2*P.L^2/3+P.m1*z^2);...
        1;...  % z*cos(theta)/(P.m2*P.L^2/3+P.m1*z^2);...
        1;...  % z*thetadot^2
        1;...  % sin(theta)
        ]);
    P.deadzone_limit = [...
        0;%2*pi/180;... % limit on theta error
        0;%0.05;...     % limit on z error
        0;%2*pi/180;... % limit on thetadot error
        0;%0.01;...     % limit on zdot error
        0;%0.1;...        % limit on integral error
        ];
    P.proj_limit = 2*[...
        1;...  %baseline controller
        1;...  % constant
        1;...  % theta
        1;...  % z-zref
        1;...  % thetadot
        1;...  % zdot
        1;...  % z*zdot*thetadot/(P.m2*P.L^2/3+P.m1*z^2);...
        1;...  % z*cos(theta)/(P.m2*P.L^2/3+P.m1*z^2);...
        1;...  % z*thetadot^2
        1;...  % sin(theta)
        ];
 % observer in the reference model
 nu = .001;
 [L,S] = lqr(P.Aref',eye(5),(nu+1)/nu*eye(5),nu/(nu+1)*eye(5));
 P.Lref_obsv = L';
 P.Pref_obsv = inv(S);
        
    


