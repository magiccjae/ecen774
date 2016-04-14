function y = position_controller(uu,P)
%
% attitude controller for hexacopter
%
persistent pc % pc => struct that will help us distinguish data for each distinct agent
              % pc(1) => data for agent 1, etc...  

% process inputs
NN = 0;
for i=1:P.num_agents,
    pc(i).pn    = uu(1+NN);  % inertial North position
    pc(i).pe    = uu(2+NN);  % inertial East position
    pc(i).pd    = uu(3+NN);  % inertial Down position
    % pc(i).u     = uu(4+NN);  % velocity in North direction
    % pc(i).v     = uu(5+NN);  % velocity in East direction
    % pc(i).w     = uu(6+NN);  % velocity in Down direction
    % pc(i).phi   = uu(7+NN);  % roll angle
    % pc(i).theta = uu(8+NN);  % pitch angle
    pc(i).psi   = uu(9+NN);  % yaw angle
    pc(i).p     = uu(10+NN); % body frame roll rate
    pc(i).q     = uu(11+NN); % body frame pitch rate
    pc(i).r     = uu(12+NN); % body frame yaw rate
    
    NN = NN + 12;
end
for i = 1:P.num_agents,
    pc(i).x_c   = uu(1+NN);  % commanded x
    pc(i).y_c   = uu(2+NN);  % commanded y
    pc(i).z_c   = uu(3+NN);  % commanded z
    pc(i).yaw_c = uu(4+NN);  % commanded yaw
    
    NN = NN + 4;
end
    t     = uu(1+NN);  % time

% define PID loops
persistent pid_x;
persistent pid_y;
persistent pid_z;
persistent pid_yaw;

% Initialize autopilot
if t==0
    y = zeros(4*P.num_agents,1); %initialize size of output
    
    for i = 1:P.num_agents
        % Initialize PID loops
        pid_x = SimplePID('pid_x', ...
            P.p_gains.Kp_x, P.p_gains.Ki_x, P.p_gains.Kd_x, ...
            P.Ts, P.tau, P.pitch_c_max);
        pid_y = SimplePID('pid_y', ...
            P.p_gains.Kp_y, P.p_gains.Ki_y, P.p_gains.Kd_y, ...
            P.Ts, P.tau, P.roll_c_max);
        pid_z = SimplePID('pid_z', ...
            P.p_gains.Kp_z, P.p_gains.Ki_z, P.p_gains.Kd_z, ...
            P.Ts, P.tau, P.F_max);
        pid_yaw = SimplePID('pid_yaw', ...
            P.p_gains.Kp_yaw, P.p_gains.Ki_yaw, P.p_gains.Kd_yaw, ...
            P.Ts, P.tau, P.yaw_rate_c_max);
    end
end

% Position controller
for i = 1:P.num_agents,
    pc(i).pitch_c    = -pid_x.computeCommand(pc(i).x_c, pc(i).pn); 
    pc(i).roll_c     =  pid_y.computeCommand(pc(i).y_c, pc(i).pe);   
    pc(i).thrust_c   = -pid_z.computeCommand(pc(i).z_c, pc(i).pd) + P.p_gains.z_ff;
    pc(i).yaw_rate_c =  pid_yaw.computeCommand(pc(i).yaw_c, pc(i).psi);

    R = [ cos(pc(i).psi),-sin(pc(i).psi),0;
          sin(pc(i).psi),cos(pc(i).psi),0;
          0,0,1];

    pc(i).rotated_c = R*[pc(i).pitch_c;pc(i).roll_c;0];

    % control outputs
    pc(i).delta = [pc(i).rotated_c(1); pc(i).rotated_c(2); pc(i).yaw_rate_c; pc(i).thrust_c];
end

NN = 0;
for i = 1:P.num_agents,
    y(NN+1:NN+4) = pc(i).delta;
    NN = NN + 4;
end

end