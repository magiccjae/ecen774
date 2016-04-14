% camera
%
% simulates camera
% input is 
%    uu(1:3) - target position
%    uu(4:6) - target velocity
%    uu(7:18) - MAV states
%
% output is 
%    px = x-pixel
%    py = y-pixel
%    ps = size of blob in image plane (in pixels)
%
% modified 11/30/2006 - Randy Beard
% modified 05/04/2007 - Randy Beard
% modified 05/04/2010 - Randy Beard
%
function out = point_gimbal(uu,P)
  
    out = zeros(2*P.num_agents,1); % initialize size of output

    % process inputs
    NN = 0;
    
    persistent theta_hat;
    if isempty(theta_hat)
        theta_hat = 0;
    end
    for i = 1:P.num_agents,
        pn(i)    = uu(1+NN); % MAV North position
        pe(i)    = uu(2+NN); % MAV East position
        pd(i)    = uu(3+NN); % MAV Down position
        u_uav(i)     = uu(4+NN); % MAV u
        v_uav(i)     = uu(5+NN); % MAV v
        w_uav(i)     = uu(6+NN); % MAV w
        phi(i)   = uu(7+NN); % MAV roll angle
        theta(i) = uu(8+NN); % MAV pitch angle
        psi(i)   = uu(9+NN); % MAV yaw angle
        p(i)     = uu(10+NN); %MAV p
        q(i)     = uu(11+NN); %MAV q
        r(i)     = uu(12+NN); %MAV r
        NN = NN + 12;
    end
    
    for i = 1:P.num_agents,
        eps_x(i)    = uu(1+NN); % x coordinate of the target in camera frame
        eps_y(i)    = uu(2+NN); % y coordinate of the target in camera frame
        eps_s(i)    = uu(3+NN); % size 
        NN = NN + 3;
    end
    
    for i = 1:P.num_agents,
        az(i)    = uu(1+NN); % gimbal azimuth angle
        el(i)    = uu(2+NN); % gimbal elevation angle
        NN = NN + 2;
    end
   
    for i = 1:P.num_agents,
        
        alpha = 5;
        Rot_g_to_c = [0 1 0; 0 0 1; 1 0 0];
        v = Rot_g_to_c * Rot_b_to_g(az(i),el(i)) * [u_uav(i); v_uav(i); w_uav(i)];  % translational velocity of camera
        vx = v(1);
        vy = v(2);
        vz = v(3);
        lambda = P.f * P.pixel_to_meter;
        gamma = 0.1;
        u = eps_x(i) * P.pixel_to_meter;
        w = eps_y(i) * P.pixel_to_meter;

        
        
%         first attemp
%         phi = [-lambda*vx + u*vz; -lambda*vy + w*vz];
%         theta_hat = theta_hat + P.Ts * (1/gamma * [u w] * phi);
%         temp_denominator = lambda^2 + u^2 + w^2;
%         L_w_inv = [0 lambda/temp_denominator;...
%                    -lambda/temp_denominator 0;...
%                    w/temp_denominator -u/temp_denominator];
%         K = 1;
%         
%         control_input = L_w_inv * (-theta_hat*phi - K*[u; w]);
%         u_az(i) = -control_input(3)/cos(el(i));
%         u_el(i) = -control_input(2);
%         
%         w_gimbal = Rot_b_to_g(az,el) * [p; q; r];   % angular velocity of gimbal
%         w_ex = w_gimbal(1);         % angular velocity of camera along the optical axis
%         u = eps_x(i) * P.pixel_to_meter;
%         w = eps_y(i) * P.pixel_to_meter;
%         z = 30;
%         
%         w_el_ref = alpha*w*lambda/(lambda^2+u^2+w^2) - w_ex*u/lambda - (lambda^2*vy-lambda*w*vz-u*w*vx+u^2*vy)/(z*(lambda^2+u^2+w^2));
%         w_cel_ref = -alpha*u*lambda/(lambda^2+u^2+w^2) - w_ex*w/lambda + (lambda^2*vx-lambda*u*vz-w*u*vy+w^2*vx)/(z*(lambda^2+u^2+w^2));
% %         w_el_ref = alpha*w*lambda/(lambda^2+u^2+w^2) - w_ex*u/lambda;
% %         w_cel_ref = -alpha*u*lambda/(lambda^2+u^2+w^2) - w_ex*w/lambda; 
%         
%         azimuth = az(i)*180/pi;
%         elevator = el(i)*180/pi;
%         
%         u_az(i) = -w_cel_ref/cos(el(i));
%         if u_az(i) > 2
%             u_az(i) = 2;
%         elseif u_az(i) < -2
%             u_az(i) = -2;
%         end
%         u_el(i) = -w_el_ref;
%         
%         tellme = [azimuth elevator u_az(i) u_el(i)]
        
    end
    
    
    % create output
    NN = 0;
    for i = 1:P.num_agents,
        out(NN+1:NN+2) = [u_az(i); u_el(i)];
        NN = NN + 2;
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
        cos(theta), 0, -sin(theta);...
        0, 1, 0;...
        sin(theta), 0, cos(theta);...
        ];
    
    Rot_v2_to_b = [...
        1, 0, 0;...
        0, cos(phi), sin(phi);...
        0, -sin(phi), cos(phi);...
        ];
    
    R = Rot_v2_to_b * Rot_v1_to_v2 * Rot_v_to_v1;

end

%%%%%%%%%%%%%%%%%%%%%%%
function R = Rot_b_to_g(az,el)
    % Rotation matrix from body coordinates to gimbal coordinates
    Rot_b_to_g1 = [...
        cos(az), sin(az), 0;...
        -sin(az), cos(az), 0;...
        0, 0, 1;...
        ];

    Rot_g1_to_g = [...
        cos(el), 0, -sin(el);...
        0, 1, 0;...
        sin(el), 0, cos(el);...
        ];

    R = Rot_g1_to_g * Rot_b_to_g1;

end
