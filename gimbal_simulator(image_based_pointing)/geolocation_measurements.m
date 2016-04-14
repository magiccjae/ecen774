% geolocation--
% Compute location of target given position in camera, this code will not
% use any type of kalman filter. it will solve for the angle in a brute
% force manner, calculating solely on geometry. The output of this function
% will be treated as measurements into RRANSAC.
%
% input is 
%    uu(1:3)   - camera data (eps_x, eps_y, eps_s)
%    uu(4:15)  - MAV states
%    uu(16:17) - gimbal azimuth, elevation 
%    uu(18)    - time
%
% output is 
%    tn - measurement North postion of target
%    te - measurement East position of target
%
function out = geolocation_measurements(in,P)
    
    persistent g; %this struct will help us to keep data separate/organized for each agent
    
    %% process inputs
    NN = 0;
    for i = 1:P.num_agents,
        g(i).numMeas = in(1+NN); NN = NN + 1;     % the first input is the number of measurements for that agent
        g(i).eps_x = [];
        g(i).eps_y = [];
        for j = 1:g(i).numMeas,
            g(i).eps_x(j)     = in(1+NN); % x-pixel
            g(i).eps_y(j)     = in(2+NN); % y-pixel
            NN = NN + 2;
        end
        NN = i*(P.maxMeasurements*2 + 1); 
    end
    for i = 1:P.num_agents,
        g(i).pn        = in(1+NN);
        g(i).pe        = in(2+NN);
        g(i).pd        = in(3+NN);
        g(i).phi       = in(7+NN);
        g(i).theta     = in(8+NN);
        g(i).psi       = in(9+NN);
        NN = NN + 12;
    end
    for i = 1:P.num_agents,
        g(i).az        = in(NN+3); % gimbal azimuth angle
        g(i).el        = in(NN+2); % gimbal elevation angle, for some reason we need to negate el
        g(i).roll      = in(NN+1); % gimbal roll angle
        NN = NN + 3;
    end
    t                  = in(1+NN); % time
    
    %% initialize size of output
    if t==0,   
        %contains tn and te. td is assumed to be 0 (on the ground)
        for i = 1:P.num_agents,
            g(i).out = zeros((P.maxMeasurements*2+1), 1); 
        end
    end
    
    %% calculate position of target
    for i = 1:P.num_agents,
        NN = 0;
        g(i).out = zeros((P.maxMeasurements*2+1), 1); % reset output
        if(g(i).numMeas > 0),
            g(i).out(NN+1) = g(i).numMeas; NN = NN + 1; % first indicate # of measurements
            for j = 1:g(i).numMeas,
                % calc tn and te
                g(i).out(NN+1:NN+2) = calcPosition(g(i).pn,g(i).pe,g(i).pd,g(i).phi,g(i).theta,g(i).psi,...
                                    g(i).az,g(i).el,g(i).roll,g(i).eps_x(j),g(i).eps_y(j),P.f(i));
                NN = NN + 2;
            end
            NN = P.maxMeasurements*2 + i; 
        end
    end
    
    out = [];
    for i = 1:P.num_agents,
        out = [out; g(i).out];
        NN = 1;
        for j = 1:g(i).numMeas,
%             if(out(NN+2)~=-9999 && out(NN+1)~=-9999),
            if(out(NN+1)>-15 && out(NN+1)<22 && out(NN+2)<0 && out(NN+2)>-38),
%                  plot(out(NN+2),out(NN+1),'.') % plot x (east) and y (north) positions
            end
            NN = NN + 2;                
        end
    end
end

%%
%%----------Functions--------%%
function [xhat] = calcPosition(pn,pe,pd,phi,theta,psi,az,el,roll,eps_x,eps_y,f)
    Rot_c_to_g = [0 1 0;0,0,1;1,0,0]';
    l_cup_c = [eps_x;eps_y;f]/sqrt(eps_x^2+eps_y^2+f^2); %eqn 13.9 in UAV book

    % calculate vector pointing from mav to target. note
    % that gimbal measurements are already in inertial frame so we don't
    % need to know the agent's attitude like in the UAV book. the only
    % attitude measurement we need is at the initial time, since the gimbal
    % was aligned with the Y6 at the start. 
    ell_i = Rot_b_to_g(az,el,roll)'*Rot_c_to_g*l_cup_c;
    ell_i = ell_i/norm(ell_i); % normalize

    %calculate position of target
    temp.theta = acos(dot(ell_i,[0;0;-1])/norm(ell_i)); %angle between ell_i and vertical axis
    if(abs(temp.theta)<90*pi/180), % only consider measurements that are pointing down, towards the earth
        temp.thetaX = acos(dot(ell_i(1:2),[0;1])/norm(ell_i(1:2))); 
        temp.thetaY = acos(dot(ell_i(1:2),[1;0])/norm(ell_i(1:2)));
        d = abs(pd)*tan(temp.theta); %distance from projection of mav onto x-y plane and target
        xhat(1) = pn + d*cos(temp.thetaY); %tn
        xhat(2) = pe + d*cos(temp.thetaX); %te
    else
        xhat(1) = -9999;
        xhat(2) = -9999;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%
function R = Rot_v_to_b(phi,theta,psi)
    % Rotation matrix from body coordinates to vehicle coordinates

    Rot_v_to_v1 = [...
        cos(psi), sin(psi), 0;...
        -sin(psi), cos(psi), 0;...
        0, 0, 1;...
        ];
    
    Rot_v1_to_v2 = [...
        cos(theta), 0, sin(theta);... % changed signs, diff from original code
        0, 1, 0;...
        -sin(theta), 0, cos(theta);...
        ];
    
    Rot_v2_to_b = [...              % changed signs, diff from original code
        1, 0, 0;...
        0, cos(phi), -sin(phi);...
        0, sin(phi), cos(phi);...
        ];
    
    R = Rot_v2_to_b * Rot_v1_to_v2 * Rot_v_to_v1;

end

%%%%%%%%%%%%%%%%%%%%%%%
function R = Rot_b_to_g(az,el,roll)
    % Rotation matrix from body coordinates to gimbal coordinates
    Rot_b_to_g1 = [...
        cos(az), sin(az), 0;...
        -sin(az), cos(az), 0;...
        0, 0, 1;...
        ];

    Rot_g1_to_g = [...
        cos(el), 0, sin(el);...
        0, 1, 0;...
        -sin(el), 0, cos(el);...
        ];
    
    % original code did not account for roll of gimbal
    Rot_roll = [...
        1, 0, 0;...
        0, cos(roll), -sin(roll);...
        0, sin(roll), cos(roll);...
        ];

    R = Rot_roll * Rot_g1_to_g * Rot_b_to_g1;

end