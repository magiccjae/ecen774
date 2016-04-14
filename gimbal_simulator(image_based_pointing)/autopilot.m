function y = autopilot_david(uu,P)
%
% autopilot for mavsim
%
% Modification History:
%   2/11/2010 - RWB
%   5/14/2010 - RWB
%   2/13/2014 - DOW

% process inputs
NN = 0;
%    pn       = uu(1+NN);  % inertial North position
%    pe       = uu(2+NN);  % inertial East position
h        = uu(3+NN);  % altitude
Va       = uu(4+NN);  % airspeed
%    alpha    = uu(5+NN);  % angle of attack
%    beta     = uu(6+NN);  % side slip angle
phi      = uu(7+NN);  % roll angle
theta    = uu(8+NN);  % pitch angle
chi      = uu(9+NN);  % course angle
p        = uu(10+NN); % body frame roll rate
q        = uu(11+NN); % body frame pitch rate
r        = uu(12+NN); % body frame yaw rate
%    Vg       = uu(13+NN); % ground speed
%    wn       = uu(14+NN); % wind North
%    we       = uu(15+NN); % wind East
%    psi      = uu(16+NN); % heading
NN = NN+16;
Va_c     = uu(1+NN);  % commanded airspeed (m/s)
h_c      = uu(2+NN);  % commanded altitude (m)
chi_c    = uu(3+NN);  % commanded course (rad)
NN = NN+3;
t        = uu(1+NN);   % time

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_with_throttle_hold - regulate airspeed using throttle
%    function delta_t = airspeed_with_throttle_hold(Va_c, Va, flag, P)
% airspeed_with_pitch_hold  - regulate airspeed using pitch angle
%    function theta_c = airspeed_with_pitch_hold(Va_c, Va, flag, P)
% altitude_hold  - regulate altitude using pitch angle
%    function theta_c = altitude_hold(h_c, h, flag, P)
% pitch_hold  - regulate pitch using elevator
%    function delta_e = pitch_hold(theta_c, theta, q, flag, P)
% roll_hold  - regulate roll using aileron
%    function delta_a = roll_hold(phi_c, phi, p, flag, P)
% course_hold  - regulate heading using the roll command
%    function phi_c = course_hold(chi_c, chi, r, flag, P)
% coordinated_turn_hold  - sideslip with rudder
%    function delta_r = coordinated_turn_hold(v, flag, P)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define state of altitude state machine
persistent fsm;

% define PID loops
persistent pid_airspeed_throttle;
persistent pid_airspeed_pitch;
persistent pid_altitude_pitch;
persistent pid_pitch;
persistent pid_roll;
persistent pid_course;
persistent pid_turn;

% Initialize autopilot
if t==0
    % Initialize PID loops
    pid_airspeed_throttle = SimplePID('pid_airspeed_throttle', ...
        P.airspeed_throttle_kp, P.airspeed_throttle_ki, P.airspeed_throttle_kd, ...
        P.Ts, P.tau, P.max_throttle);
    pid_airspeed_pitch = SimplePID('pid_airspeed_pitch', ...
        P.airspeed_pitch_kp, P.airspeed_pitch_ki, P.airspeed_pitch_kd, ...
        P.Ts, P.tau, P.max_theta);
    pid_altitude_pitch = SimplePID('pid_altitude_pitch', ...
        P.altitude_kp, P.altitude_ki, P.altitude_kd, ...
        P.Ts, P.tau, P.max_altitude);
    pid_pitch = SimplePID('pid_pitch', ...
        P.pitch_kp, P.pitch_ki, P.pitch_kd, ...
        P.Ts, P.tau, P.max_theta);
    pid_roll = SimplePID('pid_roll', ...
        P.roll_kp, P.roll_ki, P.roll_kd, ...
        P.Ts, P.tau, P.max_roll);
    pid_course = SimplePID('pid_course', ...
        P.course_kp, P.course_ki, P.course_kd, ...
        P.Ts, P.tau, P.max_course);
    pid_turn = SimplePID('pid_turn', ...
        P.beta_kp, P.beta_ki, P.beta_kd, ...
        P.Ts, P.tau, P.max_delta_r);
    
    fsm = AltitudeState.Takeoff;
    fsm.initialize(P.altitude_take_off_zone,P.altitude_hold_zone);
end

% Todo: reset integrators
current_state = fsm.getState(h,h_c);
if t==0
     h
     h_c
     fsm.take_off_zone
     current_state
end

% Set throttle and pitch
switch(current_state)
    case AltitudeState.Descend
        %disp('Descend');
        delta_t = 0;
        theta_c = pid_airspeed_pitch.computeCommand(Va_c, Va);
    case AltitudeState.Hold
        %disp('Hold');
        delta_t = pid_airspeed_throttle.computeCommand(Va_c, Va);
        theta_c = pid_altitude_pitch.computeCommand(h_c,h);
    case AltitudeState.Climb
        %disp('Climb');
        delta_t = P.max_throttle;
        theta_c = pid_airspeed_pitch.computeCommand(Va_c, Va);
    case AltitudeState.Takeoff
        %disp('Takeoff');
        delta_t = P.max_throttle;
        theta_c = P.theta_takeoff;
end


% Lateral autopilot
phi_c   = pid_course.computeCommand(chi_c, chi); % Use commanded roll angle to regulate heading
delta_a = pid_roll.computeCommand(phi_c, phi);   % Use aileron to regulate roll angle
delta_r = 0;                                     % Assume no rudder, therefore set delta_r=0
%delta_r = pid_turn.computeCommand(0,beta_c);

% Longitudinal autopilot
delta_e = pid_pitch.computeCommand(theta_c, theta);

% control outputs
delta = [delta_e; delta_a; delta_r; delta_t];
% commanded (desired) states
x_command = [...
    0;...                    % pn
    0;...                    % pe
    h_c;...                  % h
    Va_c;...                 % Va
    0;...                    % alpha
    0;...                    % beta
    phi_c;...                % phi
    theta_c*P.K_theta_DC;... % theta
    chi_c;...                % chi
    0;...                    % p
    0;...                    % q
    0;...                    % r
    ];

y = [delta; x_command];

end