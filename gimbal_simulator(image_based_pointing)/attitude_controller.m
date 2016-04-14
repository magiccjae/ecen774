function y = attitude_controller(uu,P)
%
% attitude controller for hexacopter
%
y = zeros(4*P.num_agents,1); % initialize size of output

persistent ac  % ac => struct that will help us distinguish data for each distinct agent
               % ac(1) => data for agent 1, etc...  

% process inputs
NN = 0;
for i = 1:P.num_agents,
    % ac(i).pn    = uu(1+NN);  % inertial North position
    % ac(i).pe    = uu(2+NN);  % inertial East position
    % ac(i).pd    = uu(3+NN);  % inertial Down position
    % ac(i).u     = uu(4+NN);  % velocity in North direction
    % ac(i).v     = uu(5+NN);  % velocity in East direction
    % ac(i).w     = uu(6+NN);  % velocity in Down direction
    ac(i).phi   = uu(7+NN);  % roll angle
    ac(i).theta = uu(8+NN);  % pitch angle
    % ac(i).psi   = uu(9+NN);  % yaw angle
    % ac(i).p     = uu(10+NN); % body frame roll rate
    % ac(i).q     = uu(11+NN); % body frame pitch rate
    ac(i).r     = uu(12+NN); % body frame yaw rate
    
    NN = NN + 12;
end

for i = 1:P.num_agents,
    ac(i).roll_c     = uu(2+NN);  % commanded roll (rad)
    ac(i).pitch_c    = uu(1+NN);  % commanded pitch (rad)
    ac(i).yaw_rate_c = uu(3+NN);  % commanded yaw_rate (rad/s)
    ac(i).thrust_c   = uu(4+NN);  % commanded thrust (thrust)

    NN = NN+4;
end
t        = uu(1+NN);   % time

% define PID loops
persistent pid_roll;
persistent pid_pitch;
persistent pid_yaw_rate;

% Initialize autopilot
if t==0
    % Initialize PID loops
    pid_roll = SimplePID('pid_roll', ...
        P.a_gains.Kp_roll, P.a_gains.Ki_roll, P.a_gains.Kd_roll, ...
        P.Ts, P.tau, P.T_phi_max);
    pid_pitch = SimplePID('pid_pitch', ...
        P.a_gains.Kp_pitch, P.a_gains.Ki_pitch, P.a_gains.Kd_pitch, ...
        P.Ts, P.tau, P.T_theta_max);
    pid_yaw_rate = SimplePID('pid_yaw_rate', ...
        P.a_gains.Kp_yaw_rate, P.a_gains.Ki_yaw_rate, P.a_gains.Kd_yaw_rate, ...
        P.Ts, P.tau, P.T_psi_max);
end

% Attitude control
for i = 1:P.num_agents,
    ac(i).T_phi   = pid_roll.computeCommand(ac(i).roll_c, ac(i).phi); 
    ac(i).T_theta = pid_pitch.computeCommand(ac(i).pitch_c, ac(i).theta);   
    ac(i).T_psi   = pid_yaw_rate.computeCommand(ac(i).yaw_rate_c, ac(i).r);
    ac(i).F       = ac(i).thrust_c;

    % control outputs
    ac(i).delta = [ac(i).F; ac(i).T_phi; ac(i).T_theta; ac(i).T_psi];
end

NN = 0;
for i = 1:P.num_agents,
	y(NN+1:NN+4) = ac(i).delta;
    NN = NN + 4;
end